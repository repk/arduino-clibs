CC = avr-gcc
OBJCOPY = avr-objcopy
MAKE = make

ARDUINO_LIBDIR ?= ../lib/
CORE = $(ARDUINO_LIBDIR)/core/
SPI = $(ARDUINO_LIBDIR)/spi/

ARDUINO_VARIANT ?= standard

DEPS= $(CORE) $(SPI)

CPU= -mmcu=atmega328
CFLAGS= $(CPU) -DF_CPU=16000000 -W -Wall -Werror -I$(ARDUINO_LIBDIR)	\
	-I$(ARDUINO_LIBDIR)/core/variants/$(ARDUINO_VARIANT) -Os

LDFLAGS= $(CPU) -Wl,--gc-sections -ffunction-sections -fdata-sections \
	 -static -L$(SPI) -lspi -L$(CORE) -lcore

SRC= main.c
OBJ= $(SRC:.c=.o)


EXEC= echo.elf
HEX= echo.hex

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
	rm -rf $(EXEC)
	rm -rf $(HEX)

