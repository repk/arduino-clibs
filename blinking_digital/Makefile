CC= avr-gcc
OBJCOPY= avr-objcopy
MAKE= make

ARDUINO_LIBDIR?= ../lib
CORE= $(ARDUINO_LIBDIR)/core/

DEPS= $(CORE)

CPU= -mmcu=atmega328
CFLAGS= $(CPU) -DF_CPU=16000000 -W -Wall -Werror -I$(ARDUINO_LIBDIR) -Os
LDFLAGS= $(CPU) -Wl,--gc-sections -ffunction-sections -fdata-sections \
	 -static -L$(CORE) -lcore

SRC= hello.c
OBJ= $(SRC:.c=.o)


EXEC= hello.elf
HEX= hello.hex

all: deps $(HEX)
	@echo
	@echo "Compilation success - You can now use : "
	@echo "	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:$(HEX)"

$(HEX): $(EXEC)
	$(OBJCOPY) -O ihex -R .eeprom $< $@ 

$(EXEC): $(OBJ) deps
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: clean mrproper deps

deps:
	$(foreach dir, $(DEPS), $(MAKE) -C $(dir);)

clean:
	$(foreach dir, $(DEPS), $(MAKE) -C $(dir) clean;)
	rm -rf *.o

mrproper: clean
	rm -rf $(LIB)
	rm -rf $(EXEC)
	rm -rf $(HEX)

