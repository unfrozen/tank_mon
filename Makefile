
SDCC = sdcc -mstm8 -I ../libs -L ../libs -DSTM8103

LIBS = lib_stm8.lib
OBJS = 

all: tank_mon.ihx
tank_mon.ihx: $(OBJS)

.SUFFIXES : .rel .c .ihx

.rel.ihx : $(OBJS)
	$(SDCC) $< $(LIBS) $(OBJS)

.c.rel :
	$(SDCC) -c $<
clean:
	- rm -f *.adb *.asm *.cdb *.ihx *.lk *.lst *.map *.rel *.rst *.sym

