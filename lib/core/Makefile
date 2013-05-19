CC= avr-gcc
AR= ar

ARDUINO_VARIANT ?= standard

CFLAGS= -DF_CPU=16000000 -mmcu=atmega328 -W -Wall -Werror \
	-I./variants/$(ARDUINO_VARIANT) -Os
LDFLAGS=

SRC=	malloc.c \
	WInterrupts.c \
	wiring_analog.c \
	wiring.c \
	wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c


LIB= libcore.a
OBJ= $(SRC:.c=.o)



all: $(LIB)

$(LIB): $(OBJ)
	$(AR) rcs $@ $(OBJ)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: clean mrproper

clean:
	rm -rf *.o

mrproper: clean
	rm -rf $(LIB)

