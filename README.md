# daisy-rust-playground

A working prototype for Daisy Seed Rev 5.

I am developing [`lyd`](https://github.com/chaosprint/lyd), a rust audio lib, and it can now make some basic sound.

An experimental idea is [`ljud`](https://github.com/chaosprint/ljud) which uses Box and dyn Trait.

## Usage

1. install rust
make sure you have the right version:
```
rustc -V    
```
> ```rustc 1.64.0 (a55dd71d5 2022-09-19)```

2. add the target:

```
rustup target add thumbv7em-none-eabihf
```

3. install tools:

https://github.com/rust-embedded/cargo-binutils

https://dfu-util.sourceforge.net/

4. build:
```
cargo objcopy --release -- -O binary a.bin 
```
5. connect daisy seed; hold [boot] and click on [reset]; then run the code below:
```
dfu-util -a 0 -s 0x08000000 -D a.bin
```

## Disclaimer

This is based on:

https://github.com/backtail/daisy-blank

https://github.com/backtail/libdaisy-rust

and the examples in:

https://github.com/stm32-rs/stm32h7xx-hal/

Tested on daisy seed rev 5.