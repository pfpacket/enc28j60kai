obj-m := enc28j60kai.o
KDIR := /lib/modules/$(shell uname -r)/build
VERBOSE = 0

all:
	$(MAKE) -C $(KDIR) M=$(PWD) KBUILD_VERBOSE=$(VERBOSE) CONFIG_DEBUG_INFO=y modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

insmod: all
	sudo insmod $(basename ${obj-m}).ko

rmmod:
	sudo rmmod $(basename ${obj-m})
