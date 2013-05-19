#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdlib.h>

/*
 * Define constants and variables for buffering incoming serial data.  We're
 * using a ring buffer, in which head is the index of the location to which
 * to write the next incoming character and tail is the index of the location
 * from which to read.
 */
#if (RAMEND < 1000)
#define SERIAL_BUFFER_SIZE 16
#else
#define SERIAL_BUFFER_SIZE 64
#endif


/*
 * The Ring buffer structure
 */
struct sring_buffer
{
	unsigned char buffer[SERIAL_BUFFER_SIZE];
	volatile unsigned int head;
	volatile unsigned int tail;
};


/*
 * The UART serial handling object
 */
struct serial {
	struct sring_buffer *rx_buffer;
	struct sring_buffer *tx_buffer;
	volatile uint8_t *ubrrh;
	volatile uint8_t *ubrrl;
	volatile uint8_t *ucsra;
	volatile uint8_t *ucsrb;
	volatile uint8_t *ucsrc;
	volatile uint8_t *udr;
	uint8_t rxen;
	uint8_t txen;
	uint8_t rxcie;
	uint8_t udrie;
	uint8_t u2x;
	uint8_t transmitting;
};


#if defined(UBRR3H)
extern struct serial serial[4];
#elif defined(UBRR2H)
extern struct serial serial[3];
#elif defined(UBRR1H)
extern struct serial serial[2];
#elif defined(UBRRH) && defined(UBRRL)
extern struct serial serial[1];
#elif defined(UBRR0H) && defined(UBRR0L)
extern struct serial serial[1];
#else
  #error no serial port defined  (port 0)
#endif


void serial_init(void);
void serial_begin(struct serial *s, unsigned long baud);
void serial_end(struct serial *s);
unsigned int serial_avail(struct serial *s);
int serial_peek(struct serial *s);
int serial_readchar(struct serial *s, char *c);
size_t serial_read(struct serial *s, void *buf, size_t count);
void serial_flush(struct serial *s);
int serial_writechar(struct serial *s, uint8_t c);
int serial_write(struct serial *s, void const *buf, size_t nb);

#endif
