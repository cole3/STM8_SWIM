ifneq ($(KERNELRELEASE),)

obj-m := stm8_swim.o

else
	
KDIR := /home/cole3/src/linux-2.6.36
all:
	make -C $(KDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-linux-
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order

endif
