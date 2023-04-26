rm -f a.bin
cargo objcopy --release -- -O binary a.bin 
dfu-util -a 0 -s 0x08000000 -D a.bin