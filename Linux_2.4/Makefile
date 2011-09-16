
MCFLAGS = -D__KERNEL__ -Wall -Wstrict-prototypes -O2 -fomit-frame-pointer \
	-fno-strict-aliasing -pipe -fno-strength-reduce -march=i386  -DMODULE \
	  -I /usr/src/linux-2.4/include -DEXPORT_SYMTAB
MCFLAGS1 = -D__KERNEL__ -g -pipe -m486 
CFLAGS = -o
MPATH = /lib/modules
INCLUDES = -I.
all: multiport.o 

multiport.o: multi.o 
	ld -r -o multiport.o multi.o

multi.o: multi.c 
	$(CC) $(MCFLAGS)  -c multi.c 

clean: 
	rm -f *.o

