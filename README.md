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

3. install toos:

https://github.com/rust-embedded/cargo-binutils
https://dfu-util.sourceforge.net/

4. build and run:
```
cargo objcopy --release -- -O binary a.bin 
```
```
dfu-util -a 0 -s 0x08000000 -D a.bin
```