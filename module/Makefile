MODULE_NAME := bmp280

obj-m := bmp280.o

KDIR ?= $(KERNELDIR)
CROSS_COMPILE ?= $(CC)
ARCH ?= arm
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

.PHONY: all clean