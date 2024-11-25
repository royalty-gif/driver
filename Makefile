
BOARD ?=
FILE ?=

KERN_DIR = ~/rk3568/linux_sdk/kernel
export ARCH=arm64
export CROSS_COMPILE=~/rk3568/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-


all:
	make -C $(KERN_DIR) M=`pwd` modules 

clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order *.o *.mod.c

obj-m	+= $(FILE:c=o)
