// based on https://github.com/backtail/libdaisy-rust/blob/master/src/audio.rs

#![no_main]
#![no_std]

use stm32h7xx_hal as hal;
use hal::{
    device,
    dma,
    i2c::*,
    prelude::*,
    pac,
    pac::interrupt,
    rcc,
    rcc::rec::Sai1ClkSel,
    sai,
    sai::{I2sUsers, SaiChannel, SaiI2sExt},
    stm32,
    stm32::rcc::d2ccip1r::SAI1SEL_A,
    time::{Hertz, MegaHertz},
    traits::i2s::FullDuplex,
};
use num_enum::IntoPrimitive;
use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;

mod logger;
// use core::panic::PanicInfo;
// #[panic_handler]
// fn panic(_info: &core::panic::PanicInfo) -> ! {
//     loop {}
// }

// Process samples at 1000 Hz
// With a circular buffer(*2) in stereo (*2)
pub const BLOCK_SIZE_MAX: usize = 1024;
pub const DMA_BUFFER_SIZE: usize = BLOCK_SIZE_MAX * 2 * 2;

// N samples * 2 audio channels * 2 buffers
const DMA_BUFFER_LENGTH: usize = BLOCK_SIZE_MAX * 2 * 2;
// const AUDIO_SAMPLE_HZ: Hertz = Hertz(48_000);
const AUDIO_SAMPLE_HZ: Hertz = Hertz(48_000);

pub type DmaBuffer = [u32; DMA_BUFFER_SIZE];
const START_OF_DRAM2: u32 = 0x30000000;
const DMA_MEM_SIZE: usize = 32 * BLOCK_SIZE_MAX;
const CLOCK_RATE_HZ: Hertz = Hertz(480_000_000_u32);
const MEMFAULTENA: u32 = 1 << 16;
const REGION_FULL_ACCESS: u32 = 0x03;
const REGION_ENABLE: u32 = 0x01;

const HSE_CLOCK_MHZ: MegaHertz = MegaHertz(16);
// const HCLK_MHZ: Hertz = Hertz::MHz(200);
// const HCLK2_MHZ: Hertz = Hertz::MHz(200);

// PCLKx
const PCLK_HZ: Hertz = Hertz(CLOCK_RATE_HZ.0 / 4);
// 49_152_344
// PLL1
const PLL1_P_HZ: Hertz = CLOCK_RATE_HZ;
const PLL1_Q_HZ: Hertz = Hertz(CLOCK_RATE_HZ.0 / 18);
const PLL1_R_HZ: Hertz = Hertz(CLOCK_RATE_HZ.0 / 32);
// PLL2
// const PLL2_P_HZ: Hertz = Hertz(4_800_000);
// const PLL2_Q_HZ: Hertz = Hertz(PLL2_P_HZ.0 / 2); // No divder given, what's the default?
// const PLL2_R_HZ: Hertz = Hertz(PLL2_P_HZ.0 / 4); // No divder given, what's the default?
                                                 // PLL3
                                                 // 48Khz * 256 = 12_288_000
const PLL3_P_HZ: Hertz = Hertz(AUDIO_SAMPLE_HZ.0 * 257);
const PLL3_Q_HZ: Hertz = Hertz(PLL3_P_HZ.0 / 4);
const PLL3_R_HZ: Hertz = Hertz(PLL3_P_HZ.0 / 16);

// = static data ==============================================================

// #[link_section = ".sram3"]
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
// pub const MAX_TRANSFER_SIZE: usize = BLOCK_SIZE_MAX * 2;

fn log2minus1(sz: u32) -> u32 {
    for x in 5..=31 {
        if sz == (1 << x) {
            return x - 1;
        }
    }
    panic!("Unknown memory region size!");
}


// type StereoIteratorHandle = fn(StereoIterator, &mut Output);

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


// = entry ====================================================================

#[entry]
fn main() -> ! {
    let dp = hal::pac::Peripherals::take().unwrap(); 
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    let ccdr = dp.RCC.constrain()
    .use_hse(HSE_CLOCK_MHZ)
    .sys_ck(CLOCK_RATE_HZ)
    .pclk1(PCLK_HZ) // DMA clock
    // PLL1
    .pll1_strategy(rcc::PllConfigStrategy::Iterative)
    .pll1_p_ck(PLL1_P_HZ)
    .pll1_q_ck(PLL1_Q_HZ)
    .pll1_r_ck(PLL1_R_HZ)
    // PLL2
    // .pll2_p_ck(PLL2_P_HZ) // Default adc_ker_ck_input
    // .pll2_q_ck(PLL2_Q_HZ)
    // .pll2_r_ck(PLL2_R_HZ)
    // PLL3
    .pll3_strategy(rcc::PllConfigStrategy::Iterative)
    .pll3_p_ck(PLL3_P_HZ)
    .pll3_q_ck(PLL3_Q_HZ)
    .pll3_r_ck(PLL3_R_HZ)
    .freeze(vos, &dp.SYSCFG);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpioh = dp.GPIOH.split(ccdr.peripheral.GPIOH);
    let mut core = device::CorePeripherals::take().unwrap();

    // info!("Setup up DMA...");
    let mpu = &mut core.MPU;
    let scb = &mut core.SCB;
    let location = START_OF_DRAM2 as *mut u32;
    let size = DMA_MEM_SIZE;
    // disable(mpu, scb);
    unsafe {
        /* Make sure outstanding transfers are done */
        cortex_m::asm::dmb();
        scb.shcsr.modify(|r| r & !MEMFAULTENA);
        /* Disable the MPU and clear the control register*/
        mpu.ctrl.write(0);
    }

    const REGION_NUMBER0: u32 = 0x00;
    const REGION_SHAREABLE: u32 = 0x01;
    const REGION_TEX: u32 = 0b001;
    const REGION_CB: u32 = 0b00;

    assert_eq!(
        size & (size - 1),
        0,
        "Memory region size must be a power of 2"
    );
    assert_eq!(
        size & 0x1F,
        0,
        "Memory region size must be 32 bytes or more"
    );

    unsafe {
        mpu.rnr.write(REGION_NUMBER0);
        mpu.rbar.write((location as u32) & !0x1F);
        mpu.rasr.write(
            (REGION_FULL_ACCESS << 24)
                | (REGION_TEX << 19)
                | (REGION_SHAREABLE << 18)
                | (REGION_CB << 16)
                | (log2minus1(size as u32) << 1)
                | REGION_ENABLE,
        );
    }

    // enable(mpu, scb);
    const MPU_ENABLE: u32 = 0x01;
    const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;

    unsafe {
        mpu.ctrl
            .modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

        scb.shcsr.modify(|r| r | MEMFAULTENA);

        // Ensure MPU settings take effect
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }

    // - configure dma1 -------------------------------------------------------
    let dma1_streams =
        dma::dma::StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

    let rx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
        unsafe { &mut RX_BUFFER };
    // dma1 stream 0
    let dma_config = dma::dma::DmaConfig::default()
        .priority(dma::config::Priority::High)
        .memory_increment(true)
        .peripheral_increment(false)
        .circular_buffer(true)
        .fifo_enable(false);

    // dma1_str0: output_stream
    let mut dma1_str0: dma::Transfer<_, _, dma::MemoryToPeripheral, _, _> =
        dma::Transfer::init(
            dma1_streams.0,
            unsafe { pac::Peripherals::steal().SAI1 }, // Channel B .dma_ch_b()
            rx_buffer,
            None,
            dma_config,
        );

    let tx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
    unsafe { &mut TX_BUFFER };

    // dma1 stream 1
    
    let dma_config = dma_config
        .transfer_complete_interrupt(true)
        .half_transfer_interrupt(true);
    
    //  dma1_str1 : input_stream
    let mut dma1_str1: dma::Transfer<_, _, dma::PeripheralToMemory, _, _> =
        dma::Transfer::init(
            dma1_streams.1,
            unsafe { pac::Peripherals::steal().SAI1 }, // Channel A .dma_ch_a()
            tx_buffer,
            None,
            dma_config,
        );

    // - configure sai ----------------------------------------------------
    // enable sai1 peripheral and set clock to pll3
    let sai1_rec = ccdr.peripheral.SAI1.kernel_clk_mux( SAI1SEL_A::PLL3_P ); //Sai1ClkSel::Pll3P

    let master_config = sai::I2SChanConfig::new(sai::I2SDir::Rx)
        .set_frame_sync_active_high(false);
    
        // configure sai for FS: 48 KHz, bits: 24, Data Format: MSB Justified, LRCK Order: Hi/Lo
    let slave_config = sai::I2SChanConfig::new(sai::I2SDir::Tx)
        .set_sync_type(sai::I2SSync::Internal)
        .set_frame_sync_active_high(false);

    // - configure pins ---------------------------------------------------

    // let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // let mut ak4556_reset = gpiob.pb11.into_push_pull_output();

    // let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let sai1_pins = (
        gpioe.pe2.into_alternate_af6(),       // MCLK_A
        gpioe.pe5.into_alternate_af6(),       // SCK_A
        gpioe.pe4.into_alternate_af6(),       // FS_A
        gpioe.pe6.into_alternate_af6(),       // SD_A
        Some(gpioe.pe3.into_alternate_af6()), // SD_B
        // gpioe.pe2.into_alternate::<6>(),       // MCLK_A
        // gpioe.pe5.into_alternate::<6>(),       // SCK_A
        // gpioe.pe4.into_alternate::<6>(),       // FS_A
        // gpioe.pe6.into_alternate::<6>(),       // SD_A
        // Some(gpioe.pe3.into_alternate::<6>()), // SD_B
    );

    // Hand off to audio module
    let mut sai1 = dp.SAI1.i2s_ch_a(
        sai1_pins,
        AUDIO_SAMPLE_HZ,
        sai::I2SDataSize::BITS_24,
        sai1_rec,
        &ccdr.clocks,
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
    // problem comes here??
    // let i2c2_pins = (gpioh.ph4.into_alternate_open_drain::<4>(), gpiob.pb11.into_alternate_open_drain::<4>());
    let i2c2_pins = (gpioh.ph4.into_alternate_af4(), gpiob.pb11.into_alternate_af4());
    let mut i2c = dp.I2C2.i2c(i2c2_pins, Hertz(100_000), ccdr.peripheral.I2C2, &ccdr.clocks);
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

    // // - reset ak4556 codec -----------------------------------------------

    // ak4556_reset.set_low();
    // asm::delay(480_000); // ~ 1ms (datasheet specifies minimum 150ns)
    // ak4556_reset.set_high();

    // - start audio ------------------------------------------------------
    // unmask interrupt handler for dma 1, stream 1
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA1_STR1);
    }

    dma1_str1.start(|_sai1_rb| {
        sai1.enable_dma(SaiChannel::ChannelA); // reverse?
    });

    dma1_str0.start(|sai1_rb| {
        sai1.enable_dma(SaiChannel::ChannelB);

        // wait until sai1's fifo starts to receive data
        // info!("sai1 fifo waiting to receive data");
        while sai1_rb.chb.sr.read().flvl().is_empty() {}
        // info!("audio started");

        sai1.enable();
        // Jump start audio
        // Each of the audio blocks in the SAI are enabled by SAIEN bit in the SAI_xCR1 register.
        // As soon as this bit is active, the transmitter or the receiver is sensitive
        // to the activity on the clock line, data line and synchronization line in slave mode.
        // In master TX mode, enabling the audio block immediately generates the bit clock for the
        // external slaves even if there is no data in the FIFO, However FS signal generation
        // is conditioned by the presence of data in the FIFO.
        // After the FIFO receives the first data to transmit, this data is output to external slaves.
        // If there is no data to transmit in the FIFO, 0 values are then sent in the audio frame
        // with an underrun flag generation.
        // From the reference manual (rev7 page 2259)
        sai1.try_send(0, 0).unwrap();
    });

    // changed, the order matters.
    core.SCB.enable_icache();
    core.SCB.enable_dcache(&mut core.CPUID);

    // - dma1 stream 1 interrupt handler --------------------------------------

    type TransferDma1Str1 = dma::Transfer<
        dma::dma::Stream1<stm32::DMA1>,
        // sai::dma::ChannelA<stm32::SAI1>,
        stm32::SAI1,
        dma::PeripheralToMemory,
        &'static mut [u32; DMA_BUFFER_LENGTH],
        dma::DBTransfer,
    >;

    static mut TRANSFER_DMA1_STR1: Option<TransferDma1Str1> = None;
    unsafe {
        TRANSFER_DMA1_STR1 = Some(dma1_str1);
        // info!(
        //     "{:?}, {:?}",
        //     &TX_BUFFER[0] as *const u32, &RX_BUFFER[0] as *const u32
        // );
    }

    #[interrupt]
    fn DMA1_STR1() {
        static mut PHASE: f32 = 0.0;
        let tx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
            unsafe { &mut TX_BUFFER };
        let rx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
            unsafe { &mut RX_BUFFER };

        let stereo_block_length = tx_buffer.len() / 2;

        if let Some(transfer) = unsafe { &mut TRANSFER_DMA1_STR1 } {
            let skip = if transfer.get_half_transfer_flag() {
                transfer.clear_half_transfer_interrupt();
                (0, stereo_block_length)
            } else if transfer.get_transfer_complete_flag() {
                transfer.clear_transfer_complete_interrupt();
                (stereo_block_length, 0)
            } else {
                return;
            };

            // pass thru
            let mut index = 0;
            
            while index < stereo_block_length {
                let tx0 = index + skip.0;
                let tx1 = tx0 + 1;
                // let rx0 = index + skip.1;
                // let rx1 = rx0 + 1;

                // tx_buffer[tx0] = rx_buffer[rx0];
                // tx_buffer[tx1] = rx_buffer[rx1];
                let mono = libm::sinf(*PHASE * 2.0 * core::f32::consts::PI);
                tx_buffer[tx0] = S24::from(mono).into();
                tx_buffer[tx1] = S24::from(mono).into();
                *PHASE += 440. / 48000.;
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