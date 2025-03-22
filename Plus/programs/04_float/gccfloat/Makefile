LD=/c/riscv/bin/riscv-none-elf-gcc 
GCC=/c/riscv/bin/riscv-none-elf-gcc
OBJ = /c/riscv/bin/riscv-none-elf-objcopy
AR = /c/riscv/bin/riscv-none-elf-ar
B2H = ../common/bin2hex
export objcopy := $(OBJ)
export bin2hex := $(B2H)

CFLAGS= -I./barebones -I. -c  -march=rv32i_zbs_zicsr -mabi=ilp32 -O0  -finline-functions -falign-functions=16 -falign-jumps=4 -falign-loops=4 -finline-limit=1000 -ffreestanding -fno-zero-initialized-in-bss -DPERFORMANCE_RUN=1 -DITERATIONS=1000
CCFLAGS= -I./barebones -I. -c  -march=rv32ic_zbs_zicsr -mabi=ilp32 -O0  -ffreestanding -fno-zero-initialized-in-bss -DPERFORMANCE_RUN=1 -DITERATIONS=1000
ARFLAGS = rcs 
all:libfloat.a

fabs.o:fabs.c
	$(GCC) $(CFLAGS) -o $@ $^
mulsf3.o:mulsf3.c
	$(GCC) $(CFLAGS) -o $@ $^
floatsisf.o:floatsisf.c 	
	$(GCC) $(CFLAGS) -o $@ $^
addsf3.o:addsf3.c
	$(GCC) $(CFLAGS) -o $@ $^
extendsfdf2.o:extendsfdf2.c
	$(GCC) $(CFLAGS) -o $@ $^
fixsfsi.o:fixsfsi.c
	$(GCC) $(CFLAGS) -o $@ $^
divdf3.o:divdf3.c
	$(GCC) $(CFLAGS) -o $@ $^
truncdfsf2.o:truncdfsf2.c
	$(GCC) $(CFLAGS) -o $@ $^
gesf2.o: gesf2.c
	$(GCC) $(CFLAGS) -o $@ $^
eqsf2.o:eqsf2.c
	$(GCC) $(CFLAGS) -o $@ $^
eqdf2.o:eqdf2.c
	$(GCC) $(CFLAGS) -o $@ $^
divsf3.o:divsf3.c
	$(GCC) $(CFLAGS) -o $@ $^
floatsidf.o:floatsidf.c
	$(GCC) $(CFLAGS) -o $@ $^
gtsf2.o:gtsf2.c
	$(GCC) $(CFLAGS) -o $@ $^
lesf2.o:lesf2.c
	$(GCC) $(CFLAGS) -o $@ $^
ledf2.o:ledf2.c
	$(GCC) $(CFLAGS) -o $@ $^
subsf3.o:subsf3.c
	$(GCC) $(CFLAGS) -o $@ $^
muldi3.o:muldi3.S
	$(GCC) $(CCFLAGS) -o $@ $^


libfloat.a:floatsisf.o extendsfdf2.o fixsfsi.o mulsf3.o  divdf3.o truncdfsf2.o fabs.o gesf2.o eqsf2.o divsf3.o eqdf2.o floatsidf.o addsf3.o gesf2.o lesf2.o subsf3.o ledf2.o  muldi3.o
	$(AR) $(ARFLAGS)  -o $@ $^

clean:
	rm -rf *.o *.a