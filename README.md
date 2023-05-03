# itop4412 uboot

## Compiling environment:
### ubuntu 22.04lts
## Compiler tool chain:
### arm-none-linux-gnueabihf-gcc version 12.2.1
## Compilation steps:
### make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabihf- itop4412_defconfig
### make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabihf-
### cat E4412_N.bl1.SCP2G.bin ./spl/itop4412-spl.bin u-boot.bin > itop4412-uboot.bin
## Start from sd-card(in host Linux environment)
### sudo dd iflag=dsync oflag=dsync if=itop4412-uboot.bin of=/dev/sdb bs=512 seek=1
## Start from emmc(in target Linux environment)
### echo 0 > /sys/block/mmcblk1boot0/force_ro
### dd if=/root/itop4412-uboot.bin of=/dev/mmcblk1boot0 bs=512
