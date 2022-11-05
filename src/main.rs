// mainly based on:
// https://github.com/backtail/libdaisy-rust
// https://github.com/stm32-rs/stm32h7xx-hal/

#![no_main]
#![no_std]

use stm32h7xx_hal::{
    dma,
    gpio::{gpiob, gpioe, gpioh, Analog},
    i2c::*,
    pac, rcc,
    rcc::rec,
    sai,
    sai::*,
    stm32,
    stm32::rcc::d2ccip1r::SAI1SEL_A,
    rcc::rec::Sai1ClkSel,
    time,
    time::{Hertz, MegaHertz},
    traits::i2s::FullDuplex,
};

use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::pac::interrupt;

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use num_enum::IntoPrimitive;

mod logger;
mod mpu;
use mpu::dma_init;
// use logger::*;

// Process samples at 1000 Hz
// With a circular buffer(*2) in stereo (*2)
pub const BLOCK_SIZE_MAX: usize = 1024;
pub const DMA_BUFFER_SIZE: usize = BLOCK_SIZE_MAX * 2 * 2;

pub type DmaBuffer = [u32; DMA_BUFFER_SIZE];

const START_OF_DRAM2: u32 = 0x30000000;
const DMA_MEM_SIZE: usize = 32 * 1024;
#[link_section = ".sram1_bss"]
#[no_mangle]
static mut TX_BUFFER: DmaBuffer = [0; DMA_BUFFER_SIZE];
#[link_section = ".sram1_bss"]
#[no_mangle]
static mut RX_BUFFER: DmaBuffer = [0; DMA_BUFFER_SIZE];
const FBIPMAX: f32 = 0.999985;
const FBIPMIN: f32 = -FBIPMAX;
const F32_TO_S24_SCALE: f32 = 8388608.0; // 2 ** 23
const S24_TO_F32_SCALE: f32 = 1.0 / F32_TO_S24_SCALE;
const S24_SIGN: i32 = 0x800000;
/// Largest number of audio blocks for a single DMA operation
pub const MAX_TRANSFER_SIZE: usize = BLOCK_SIZE_MAX * 2;
pub type AudioBuffer = [(f32, f32); BLOCK_SIZE_MAX];

type DmaInputStream = dma::Transfer<
    dma::dma::Stream1<stm32::DMA1>,
    stm32::SAI1,
    dma::PeripheralToMemory,
    &'static mut [u32; DMA_BUFFER_SIZE],
    dma::DBTransfer,
>;
type DmaOutputStream = dma::Transfer<
    dma::dma::Stream0<stm32::DMA1>,
    stm32::SAI1,
    dma::MemoryToPeripheral,
    &'static mut [u32; DMA_BUFFER_SIZE],
    dma::DBTransfer,
>;

// pub struct Audio {
//     sai: sai::Sai<stm32::SAI1, sai::I2S>,
//     input: Input,
//     output: Output,
//     input_stream: DmaInputStream,
//     output_stream: DmaOutputStream,
// }


// - clocks

pub const MILLI: u32 = 1_000;
pub const AUDIO_FRAME_RATE_HZ: u32 = 1_000;
pub const AUDIO_BLOCK_SIZE: u16 = 48;
pub const AUDIO_SAMPLE_RATE: usize = 48_000;
pub const AUDIO_SAMPLE_HZ: Hertz = Hertz::from_raw(48_000);
pub const CLOCK_RATE_HZ: Hertz = Hertz::from_raw(480_000_000_u32);


const HSE_CLOCK_MHZ: Hertz = Hertz::MHz(16);
const HCLK_MHZ: MegaHertz = MegaHertz::from_raw(200);
const HCLK2_MHZ: MegaHertz = MegaHertz::from_raw(200);

// PCLKx
const PCLK_HZ: Hertz = Hertz::from_raw(CLOCK_RATE_HZ.raw() / 4);
// 49_152_344
// PLL1
const PLL1_P_HZ: Hertz = CLOCK_RATE_HZ;
const PLL1_Q_HZ: Hertz = Hertz::from_raw(CLOCK_RATE_HZ.raw() / 18);
const PLL1_R_HZ: Hertz = Hertz::from_raw(CLOCK_RATE_HZ.raw() / 32);
// PLL2
const PLL2_P_HZ: Hertz = Hertz::from_raw(4_000_000);
const PLL2_Q_HZ: Hertz = Hertz::from_raw(PLL2_P_HZ.raw() / 2); // No divder given, what's the default?
const PLL2_R_HZ: Hertz = Hertz::from_raw(PLL2_P_HZ.raw() / 4); // No divder given, what's the default?
                                                 // PLL3
                                                 // 48Khz * 256 = 12_288_000
const PLL3_P_HZ: Hertz = Hertz::from_raw(AUDIO_SAMPLE_HZ.raw() * 257);
const PLL3_Q_HZ: Hertz = Hertz::from_raw(PLL3_P_HZ.raw() / 4);
const PLL3_R_HZ: Hertz = Hertz::from_raw(PLL3_P_HZ.raw() / 16);


// - WM8731 codec register addresses -------------------------------------------------

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, IntoPrimitive)]
#[repr(u8)]
enum Register {
    LINVOL = 0x00,
    RINVOL = 0x01,
    LOUT1V = 0x02,
    ROUT1V = 0x03,
    APANA = 0x04,
    APDIGI = 0x05, // 0000_0101
    PWR = 0x06,
    IFACE = 0x07,  // 0000_0111
    SRATE = 0x08,  // 0000_1000
    ACTIVE = 0x09, // 0000_1001
    RESET = 0x0F,
}

const REGISTER_CONFIG: &[(Register, u8)] = &[
    // reset Codec
    (Register::RESET, 0x00),
    // set line inputs 0dB
    (Register::LINVOL, 0x17),
    (Register::RINVOL, 0x17),
    // set headphone to mute
    (Register::LOUT1V, 0x00),
    (Register::ROUT1V, 0x00),
    // set analog and digital routing
    (Register::APANA, 0x12),
    (Register::APDIGI, 0x00),
    // configure power management
    (Register::PWR, 0x42),
    // configure digital format
    (Register::IFACE, 0b1001),
    // set samplerate
    (Register::SRATE, 0x00),
    (Register::ACTIVE, 0x00),
    (Register::ACTIVE, 0x01),
];

#[entry]
fn main() -> ! {

    // system init
    // let mut core: rtic::export::Peripherals = ;
    // this is different 
    let mut core = stm32h7xx_hal::device::CorePeripherals::take().unwrap();

    let device: stm32::Peripherals = pac::Peripherals::take().unwrap();

    // let pwr: stm32::PWR = device.PWR;
    
    // let rcc: stm32::RCC = device.RCC;
    
    let syscfg: &stm32::SYSCFG = &device.SYSCFG;

    // system init clocks
    let pwr = device.PWR.constrain();
    let vos = pwr.vos0(syscfg).freeze();

    let ccdr = device.RCC.constrain()
        .use_hse(HSE_CLOCK_MHZ)
        .sys_ck(CLOCK_RATE_HZ)
        .pclk1(PCLK_HZ) // DMA clock
        // PLL1
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_p_ck(PLL1_P_HZ)
        .pll1_q_ck(PLL1_Q_HZ)
        .pll1_r_ck(PLL1_R_HZ)
        // PLL2
        .pll2_p_ck(PLL2_P_HZ) // Default adc_ker_ck_input
        // .pll2_q_ck(PLL2_Q_HZ)
        // .pll2_r_ck(PLL2_R_HZ)
        // PLL3
        .pll3_strategy(rcc::PllConfigStrategy::Iterative)
        .pll3_p_ck(PLL3_P_HZ)
        .pll3_q_ck(PLL3_Q_HZ)
        .pll3_r_ck(PLL3_R_HZ)
        .freeze(vos, &syscfg);



    // info!("Setting up GPIOs...");
    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = device.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpioh = device.GPIOH.split(ccdr.peripheral.GPIOH);
    let gpioi = device.GPIOI.split(ccdr.peripheral.GPIOI);
   
    // setting up audio
    let dma1_d: stm32::DMA1 = device.DMA1;
    let dma1_p: rec::Dma1 = ccdr.peripheral.DMA1;
    let sai1_d: stm32::SAI1 = device.SAI1;
    let sai1_p: rec::Sai1 = ccdr.peripheral.SAI1;
    let i2c2_d: stm32::I2C2 = device.I2C2;
    let i2c2_p: rec::I2c2 = ccdr.peripheral.I2C2;
    
    // SAI pins
    let sai_mclk_a: gpioe::PE2<Analog> = gpioe.pe2;
    let sai_sd_b: gpioe::PE3<Analog> = gpioe.pe3;
    let sai_fs_a: gpioe::PE4<Analog> = gpioe.pe4;
    let sai_sck_a: gpioe::PE5<Analog> = gpioe.pe5;
    let sai_sd_a: gpioe::PE6<Analog> = gpioe.pe6;
    
    //I2C pins
    let i2c_scl: gpioh::PH4<Analog> = gpioh.ph4;
    let i2c_sda: gpiob::PB11<Analog> = gpiob.pb11;
    
    let clocks: &rcc::CoreClocks = &ccdr.clocks;
    let mpu: &mut cortex_m::peripheral::MPU = &mut core.MPU;
    let scb: &mut cortex_m::peripheral::SCB = &mut core.SCB;


    // info!("Setup up DMA...");
    dma_init(mpu, scb, START_OF_DRAM2 as *mut u32, DMA_MEM_SIZE);
    let dma1_streams = dma::dma::StreamsTuple::new(dma1_d, dma1_p);
    // dma1 stream 0
    let rx_buffer: &'static mut [u32; DMA_BUFFER_SIZE] = unsafe { &mut RX_BUFFER };
    let dma_config = dma::dma::DmaConfig::default()
        .priority(dma::config::Priority::High)
        .memory_increment(true)
        .peripheral_increment(false)
        .circular_buffer(true)
        .fifo_enable(false);

    let mut output_stream: dma::Transfer<_, _, dma::MemoryToPeripheral, _, _> =
    dma::Transfer::init(
        dma1_streams.0,
        unsafe { pac::Peripherals::steal().SAI1 },
        rx_buffer,
        None,
        dma_config,
    );

    // dma1 stream 1
    let tx_buffer: &'static mut [u32; DMA_BUFFER_SIZE] = unsafe { &mut TX_BUFFER };
    let dma_config = dma_config
        .transfer_complete_interrupt(true)
        .half_transfer_interrupt(true);
    let mut input_stream: dma::Transfer<_, _, dma::PeripheralToMemory, _, _> =
        dma::Transfer::init(
            dma1_streams.1,
            unsafe { pac::Peripherals::steal().SAI1 },
            tx_buffer,
            None,
            dma_config,
        );
    // info!("Setup up SAI...");
    let sai1_rec = sai1_p.kernel_clk_mux(Sai1ClkSel::Pll3P);
    let master_config = I2SChanConfig::new(I2SDir::Rx)
    .set_frame_sync_active_high(false);
    let slave_config = I2SChanConfig::new(I2SDir::Tx)
        .set_sync_type(I2SSync::Internal)
        .set_frame_sync_active_high(false);
    
    // SAI pins
    let pins_a = (
        sai_mclk_a.into_alternate_af6(),
        sai_sck_a.into_alternate_af6(),
        sai_fs_a.into_alternate_af6(),
        sai_sd_a.into_alternate_af6(),
        Some(sai_sd_b.into_alternate_af6()),
    );


    // Hand off to audio module
    let mut sai = sai1_d.i2s_ch_a(
        pins_a,
        crate::AUDIO_SAMPLE_HZ,
        I2SDataSize::BITS_24,
        sai1_rec,
        clocks,
        I2sUsers::new(master_config).add_slave(slave_config),
    );

    // Manually configure Channel B as transmit stream
    let dma1_reg = unsafe { pac::Peripherals::steal().DMA1 };
    dma1_reg.st[0]
        .cr
        .modify(|_, w| w.dir().peripheral_to_memory());

    // Manually configure Channel A as receive stream
    dma1_reg.st[1]
        .cr
        .modify(|_, w| w.dir().memory_to_peripheral());

    // info!("Setup up WM8731 Audio Codec...");
    // let i2c2_pins = (i2c_scl.into_alternate_af4(), i2c_sda.into_alternate_af4());
    let i2c2_pins = (i2c_scl.into_alternate_open_drain::<4>(), i2c_sda.into_alternate_open_drain::<4>());
    let mut i2c = i2c2_d.i2c(i2c2_pins, Hertz::from_raw(100_000), i2c2_p, clocks);

    let codec_i2c_address: u8 = 0x1a; // or 0x1b if CSB is high

    // Go through configuration setup
    for (register, value) in REGISTER_CONFIG {
        let register: u8 = (*register).into();
        let value: u8 = (*value).into();
        let byte1: u8 = ((register << 1) & 0b1111_1110) | ((value >> 7) & 0b0000_0001u8);
        let byte2: u8 = value & 0b1111_1111;
        let bytes = [byte1, byte2];

        i2c.write(codec_i2c_address, &bytes).unwrap_or_default();

        // wait ~10us
        asm::delay(5_000);
    }

    // - start audio ------------------------------------------------------

    // unmask interrupt handler for dma 1, stream 1
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA1_STR1);
    }

    // info!("Start audio stream...");
    input_stream.start(|_sai1_rb| {
        sai.enable_dma(SaiChannel::ChannelA);
    });

    output_stream.start(|sai1_rb| {
        sai.enable_dma(SaiChannel::ChannelB);

        // wait until sai1's fifo starts to receive data
        // info!("Sai1 fifo waiting to receive data.");
        while sai1_rb.chb().sr.read().flvl().is_empty() {}
        // info!("Audio started!");
        sai.enable();
        sai.try_send(0, 0).unwrap();
    });

    // audio setup done

    // init cache
    // changed
    scb.enable_icache();
    scb.enable_dcache(&mut core.CPUID);
    // - dma1 stream 1 interrupt handler --------------------------------------

    // type TransferDma1Str1 = dma::Transfer<
    //     dma::dma::Stream1<stm32::DMA1>,
    //     // sai::dma::ChannelA<stm32::SAI1>,
    //     stm32::SAI1,
    //     dma::PeripheralToMemory,
    //     &'static mut [u32; DMA_BUFFER_LENGTH],
    //     dma::DBTransfer,
    // >;

    static mut TRANSFER_DMA_INPUT_STREAM: Option<DmaInputStream> = None;
    unsafe {
        TRANSFER_DMA_INPUT_STREAM = Some(input_stream);
    }

    #[interrupt]
    fn DMA1_STR1() {
        static mut PHASE: f32 = 0.0;
        let tx_buffer: &'static mut [u32; DMA_BUFFER_SIZE] =
            unsafe { &mut TX_BUFFER };
        let rx_buffer: &'static mut [u32; DMA_BUFFER_SIZE] =
            unsafe { &mut RX_BUFFER };

        let stereo_block_length = tx_buffer.len() / 2;

        if let Some(transfer) = unsafe { &mut TRANSFER_DMA_INPUT_STREAM } {
            let skip = if transfer.get_half_transfer_flag() {
                transfer.clear_half_transfer_interrupt();
                (0, stereo_block_length)
            } else if transfer.get_transfer_complete_flag() {
                transfer.clear_transfer_complete_interrupt();
                (stereo_block_length, 0)
            } else {
                return;
            };

            let mut index = 0;
            
            while index < stereo_block_length {
                let tx0 = index + skip.0;
                let tx1 = tx0 + 1;
                let mono = libm::sinf(*PHASE * 2.0 * core::f32::consts::PI);
                tx_buffer[tx0] = S24::from(mono).into();
                tx_buffer[tx1] = S24::from(mono).into();
                *PHASE += 880. / 48000.;
                if *PHASE >= 1.0 {
                    *PHASE -= 1.0;
                }
                index += 2;
            }
        }
    }

    loop {
        asm::wfi();
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct S24(i32);

impl From<i32> for S24 {
    fn from(x: i32) -> S24 {
        S24(x)
    }
}

impl From<u32> for S24 {
    fn from(x: u32) -> S24 {
        S24(x as i32)
    }
}

impl From<S24> for i32 {
    fn from(x: S24) -> i32 {
        x.0
    }
}

impl From<S24> for u32 {
    fn from(x: S24) -> u32 {
        x.0 as u32
    }
}

impl From<f32> for S24 {
    fn from(x: f32) -> S24 {
        let x = if x <= FBIPMIN {
            FBIPMIN
        } else if x >= FBIPMAX {
            FBIPMAX
        } else {
            x
        };
        S24((x * F32_TO_S24_SCALE) as i32)
    }
}

impl From<S24> for f32 {
    fn from(x: S24) -> f32 {
        ((x.0 ^ S24_SIGN) - S24_SIGN) as f32 * S24_TO_F32_SCALE
    }
}