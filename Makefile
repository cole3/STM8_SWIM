ifneq ($(KERNELRELEASE),)

MODULE_NAME := swim

$(MODULE_NAME)-objs := stm8_swim.o main.o
obj-m := swim.o

else
	
KDIR := /home/cole3/src/linux-2.6.36

all:
	make -C $(KDIR) M=$(PWD)  modules ARCH=arm CROSS_COMPILE=arm-linux-
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order


endif