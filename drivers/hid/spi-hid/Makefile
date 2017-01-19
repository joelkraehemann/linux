ifneq ($(KERNELRELEASE),)

obj-m := spi-hid.o

else

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

spikbd_m := spi-hid.o

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

endif
