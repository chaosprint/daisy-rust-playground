## State

It's not working yet...

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