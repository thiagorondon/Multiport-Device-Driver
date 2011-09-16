    ifneq ($(KERNELRELEASE),)
	obj-m := multiport.o

    else
    KDIR	:= /lib/modules/$(shell uname -r)/build
    PWD		:= $(shell pwd)

    default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
    endif 
clean:
	rm -f *.ko
	rm -f *.o
	rm -f *.mod.o
	rm -f *.mod.c
	rm -f *.symvers
rmport:
	/sbin/rmmod multiport
insport:
	/sbin/insmod multiport.ko
