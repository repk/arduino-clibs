CC= avr-gcc
AR= ar
MAKE= make

ARDUINO_LIBS= /home/repk/Arduino/lib

CFLAGS= -DF_CPU=16000000 -mmcu=atmega328 -W -Wall -Werror -I$(ARDUINO_LIBS) -Os
LDFLAGS=

LIB= libserial.a
SRC= serial.c
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
