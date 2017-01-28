ifneq ($(KERNELRELEASE),)

obj-m := spi-hid.o hid-apple-topcase.o

else

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

spikbd_m := spi-hid.o hid-apple-topcase.o

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

endif
