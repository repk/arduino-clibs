/*
   HardwareSerial.cpp - Hardware serial library for Wiring
   Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

   Modified 23 November 2006 by David A. Mellis
   Modified 28 September 2010 by Mark Sproul
   Modified 14 August 2012 by Alarus
   */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <core/arduino.h>
#include <core/wiring_private.h>

#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) ||		\
	defined(UBRR2H) || defined(UBRR3H)

#include "serial.h"

#if defined(UBRR3H)
struct serial serial[4] = {};
#elif defined(UBRR2H)
struct serial serial[3] = {};
#elif defined(UBRR1H)
struct serial serial[2] = {};
#elif defined(UBRRH) && defined(UBRRL)
struct serial serial[1] = {};
#elif defined(UBRR0H) && defined(UBRR0L)
struct serial serial[1] = {};
#endif

/*
 * on ATmega8, the uart and its bits are not numbered, so there is no "TXC0"
 * definition.
 */
#if !defined(TXC0)
#if defined(TXC)
#define TXC0 TXC
#elif defined(TXC1)
// Some devices have uart1 but no uart0
#define TXC0 TXC1
#else
#error TXC0 not definable in HardwareSerial.h
#endif
#endif

#if defined(UBRR3H)
static struct sring_buffer rx_buffer[4]  =  {{{0}, 0, 0 }};
static struct sring_buffer tx_buffer[4]  =  {{{0}, 0, 0 }};
#elif defined(UBRR2H)
static struct sring_buffer rx_buffer[3]  =  {{{0}, 0, 0 }};
static struct sring_buffer tx_buffer[3]  =  {{{0}, 0, 0 }};
#elif defined(UBRR1H)
static struct sring_buffer rx_buffer[2]  =  { { 0 }, 0, 0 };
static struct sring_buffer tx_buffer[2]  =  { { 0 }, 0, 0 };
#elif defined(UBRRH) || defined(UBRR0H)
static struct sring_buffer rx_buffer[1]  =  {{{0}, 0, 0 }};
static struct sring_buffer tx_buffer[1]  =  {{{0}, 0, 0 }};
#else
#error "Don't know how the USART Baud rate register is called"
#endif



static inline void store_char(unsigned char c, struct sring_buffer *buffer)
{
	unsigned int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

	/*
	 * if we should be storing the received character into the location
	 * just before the tail (meaning that the head would advance to the
	 * current location of the tail), we're about to overflow the buffer
	 * and so we don't write the character or advance the head.
	 */
	if (i != buffer->tail) {
		buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}


#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(USART_RX_vect) && !defined(SIG_USART0_RECV) && \
	!defined(SIG_UART0_RECV) && !defined(USART0_RX_vect) && \
!defined(SIG_UART_RECV)
#error "Don't know what the Data Received vector is called for the first UART"
#else
void serial_event() __attribute__((weak));
void serial_event() {}
#define serial_event_implemented
/* ISR defines interrupt handler for the appropriate interrupt vector */
#if defined(USART_RX_vect)
ISR(USART_RX_vect)
#elif defined(SIG_USART0_RECV)
ISR(SIG_USART0_RECV)
#elif defined(SIG_UART0_RECV)
ISR(SIG_UART0_RECV)
#elif defined(USART0_RX_vect)
ISR(USART0_RX_vect)
#elif defined(SIG_UART_RECV)
ISR(SIG_UART_RECV)
#endif
{
	unsigned char c;
#if defined(UDR0)
	if (bit_is_clear(UCSR0A, UPE0)) {
		c = UDR0;
		store_char(c, &rx_buffer[0]);
	}
#elif defined(UDR)
	if (bit_is_clear(UCSRA, PE)) {
		c = UDR;
		store_char(c, &rx_buffer[0]);
	}
#else
#error UDR not defined
#endif
}
#endif
#endif

#if defined(USART1_RX_vect)
void serial_event1() __attribute__((weak));
void serial_event1() {}
#define serial_event1_implemented
ISR(USART1_RX_vect)
{
	unsigned char c;
	if (bit_is_clear(UCSR1A, UPE1)) {
		c = UDR1;
		store_char(c, &rx_buffer[1]);
	}
}
#elif defined(SIG_USART1_RECV)
#error SIG_USART1_RECV
#endif

#if defined(USART2_RX_vect) && defined(UDR2)
void serial_event2() __attribute__((weak));
void serial_event2() {}
#define serial_event2_implemented
ISR(USART2_RX_vect)
{
	unsigned char c;
	if (bit_is_clear(UCSR2A, UPE2)) {
		c = UDR2;
		store_char(c, &rx_buffer[2]);
	}
}
#elif defined(SIG_USART2_RECV)
#error SIG_USART2_RECV
#endif

#if defined(USART3_RX_vect) && defined(UDR3)
void serial_event3() __attribute__((weak));
void serial_event3() {}
#define serial_event3_implemented
ISR(USART3_RX_vect)
{
	unsigned char c;
	if (bit_is_clear(UCSR3A, UPE3)) {
		c = UDR3;
		store_char(c, &rx_buffer[3]);
	}
}
#elif defined(SIG_USART3_RECV)
#error SIG_USART3_RECV
#endif

void serial_event_run(void)
{
#ifdef serial_event_implemented
	if(serial_avail(&serial[0])) {
		serial_event();
	}
#endif
#ifdef serial_event1_implemented
	if(serial_avail(&serial[1])) {
		serial_event1();
	}
#endif
#ifdef serial_event2_implemented
	if(serial_avail(&serial[2])) {
		serial_event2();
	}
#endif
#ifdef serial_event3_implemented
	if(serial_avail(&serial[3])) {
		serial_event3();
	}
#endif
}


#if !defined(USART0_UDRE_vect) && defined(USART1_UDRE_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(UART0_UDRE_vect) && !defined(UART_UDRE_vect) &&		\
	!defined(USART0_UDRE_vect) && !defined(USART_UDRE_vect)
#error "Don't know what the Data Register Empty vector is called for the first UART"
#else
#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#endif
{
	if(tx_buffer[0].head == tx_buffer[0].tail) {
		/* Buffer empty, so disable interrupts */
#if defined(UCSR0B)
		cbi(UCSR0B, UDRIE0);
#else
		cbi(UCSRB, UDRIE);
#endif
	} else {
		/*
		 * There is more data in the output buffer. Send the next byte
		 */
		unsigned char c = tx_buffer[0].buffer[tx_buffer[0].tail];
		tx_buffer[0].tail = (tx_buffer[0].tail + 1) % SERIAL_BUFFER_SIZE;

#if defined(UDR0)
		UDR0 = c;
#elif defined(UDR)
		UDR = c;
#else
#error UDR not defined
#endif
	}
}
#endif
#endif

#ifdef USART1_UDRE_vect
ISR(USART1_UDRE_vect)
{
	if(tx_buffer[1].head == tx_buffer[1].tail) {
		/* Buffer empty, so disable interrupts */
		cbi(UCSR1B, UDRIE1);
	} else {
		/*
		 * There is more data in the output buffer. Send the next byte
		 */
		unsigned char c = tx_buffer[1].buffer[tx_buffer[1].tail];
		tx_buffer[1].tail = (tx_buffer[1].tail + 1) %
			SERIAL_BUFFER_SIZE;
		UDR1 = c;
	}
}
#endif

#ifdef USART2_UDRE_vect
ISR(USART2_UDRE_vect)
{
	if (tx_buffer[2].head == tx_buffer[2].tail) {
		/* Buffer empty, so disable interrupts */
		cbi(UCSR2B, UDRIE2);
	} else {
		/*
		 * There is more data in the output buffer. Send the next byte
		 */
		unsigned char c = tx_buffer[2].buffer[tx_buffer[2].tail];
		tx_buffer[2].tail = (tx_buffer[2].tail + 1) %
			SERIAL_BUFFER_SIZE;
		UDR2 = c;
	}
}
#endif

#ifdef USART3_UDRE_vect
ISR(USART3_UDRE_vect)
{
	if (tx_buffer[3].head == tx_buffer[3].tail) {
		/* Buffer empty, so disable interrupts */
		cbi(UCSR3B, UDRIE3);
	}
	else {
		/*
		 * There is more data in the output buffer. Send the next byte
		 */
		unsigned char c = tx_buffer[3].buffer[tx_buffer[3].tail];
		tx_buffer[3].tail = (tx_buffer[3].tail + 1) %
			SERIAL_BUFFER_SIZE;
		UDR3 = c;
	}
}
#endif


/* Serial structure initialization */

static inline void _serial_init(struct serial *s,
		struct sring_buffer *rx_buffer, struct sring_buffer *tx_buffer,
		volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
		volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
		volatile uint8_t *ucsrc, volatile uint8_t *udr,
		uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie,
		uint8_t u2x)
{
	s->rx_buffer = rx_buffer;
	s->tx_buffer = tx_buffer;
	s->ubrrh = ubrrh;
	s->ubrrl = ubrrl;
	s->ucsra = ucsra;
	s->ucsrb = ucsrb;
	s->ucsrc = ucsrc;
	s->udr = udr;
	s->rxen = rxen;
	s->txen = txen;
	s->rxcie = rxcie;
	s->udrie = udrie;
	s->u2x = u2x;
}

void serial_init(void)
{
#if defined(UBRRH) && defined(UBRRL)
	_serial_init(&serial[0], &rx_buffer[0], &tx_buffer[0], &UBRRH, &UBRRL,
			&UCSRA, &UCSRB, &UCSRC, &UDR, RXEN, TXEN, RXCIE, UDRIE,
			U2X);
#elif defined(UBRR0H) && defined(UBRR0L)
	_serial_init(&serial[0], &rx_buffer[0], &tx_buffer[0], &UBRR0H, &UBRR0L,
			&UCSR0A, &UCSR0B, &UCSR0C, &UDR0, RXEN0, TXEN0, RXCIE0,
			UDRIE0, U2X0);
#else
  #error no serial port defined  (port 0)
#endif

#if defined(UBRR1H)
	_serial_init(&serial[1], &rx_buffer[1], &tx_buffer[1], &UBRR1H, &UBRR1L,
			&UCSR1A, &UCSR1B, &UCSR1C, &UDR1, RXEN1, TXEN1, RXCIE1,
			UDRIE1, U2X1);
#endif
#if defined(UBRR2H)
	_serial_init(&serial[2], &rx_buffer[2], &tx_buffer[2], &UBRR2H, &UBRR2L,
			&UCSR2A, &UCSR2B, &UCSR2C, &UDR2, RXEN2, TXEN2, RXCIE2,
			UDRIE2, U2X2);
#endif
#if defined(UBRR3H)
	_serial_init(&serial[3], &rx_buffer[3], &tx_buffer[3], &UBRR3H, &UBRR3L,
			&UCSR3A, &UCSR3B, &UCSR3C, &UDR3, RXEN3, TXEN3, RXCIE3,
			UDRIE3, U2X3);
#endif
}

void serial_begin(struct serial *s, unsigned long baud)
{
	uint16_t baud_setting;
	uint8_t use_u2x = 1;

#if F_CPU == 16000000UL
	/*
	 * hardcoded exception for compatibility with the bootloader shipped
	 * with the Duemilanove and previous boards and the firmware on the 8U2
	 * on the Uno and Mega 2560.
	 */
	if(baud == 57600) {
		use_u2x = 0;
	}
#endif

try_again:

	/* TODO "x/4" or "x/2" really ... */
	if(use_u2x) {
		*s->ucsra = 1 << s->u2x;
		baud_setting = ((F_CPU >> 2) / baud - 1) >> 1;
	} else {
		*s->ucsra = 0;
		baud_setting = ((F_CPU >> 3) / baud - 1) >> 1;
	}

	if ((baud_setting > 4095) && use_u2x)
	{
		use_u2x = 0;
		goto try_again;
	}

	/* assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register) */
	*s->ubrrh = baud_setting >> 8;
	*s->ubrrl = baud_setting;

	s->transmitting = 0;

	sbi(*s->ucsrb, s->rxen);
	sbi(*s->ucsrb, s->txen);
	sbi(*s->ucsrb, s->rxcie);
	cbi(*s->ucsrb, s->udrie);
}

#if 0
void HardwareSerial::begin(unsigned long baud, byte config)
{
	uint16_t baud_setting;
	uint8_t current_config;
	bool use_u2x = true;

#if F_CPU == 16000000UL
	// hardcoded exception for compatibility with the bootloader shipped
	// with the Duemilanove and previous boards and the firmware on the 8U2
	// on the Uno and Mega 2560.
	if (baud == 57600) {
		use_u2x = false;
	}
#endif

try_again:

	if (use_u2x) {
		*_ucsra = 1 << _u2x;
		baud_setting = (F_CPU / 4 / baud - 1) / 2;
	} else {
		*_ucsra = 0;
		baud_setting = (F_CPU / 8 / baud - 1) / 2;
	}

	if ((baud_setting > 4095) && use_u2x)
	{
		use_u2x = false;
		goto try_again;
	}

	// assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
	*_ubrrh = baud_setting >> 8;
	*_ubrrl = baud_setting;

	//set the data bits, parity, and stop bits
#if defined(__AVR_ATmega8__)
	config |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
	*_ucsrc = config;

	sbi(*_ucsrb, _rxen);
	sbi(*_ucsrb, _txen);
	sbi(*_ucsrb, _rxcie);
	cbi(*_ucsrb, _udrie);
}
#endif

void serial_end(struct serial *s)
{
	/* wait for transmission of outgoing data */
	while(s->tx_buffer->head != s->tx_buffer->tail);

	cbi(*s->ucsrb, s->rxen);
	cbi(*s->ucsrb, s->txen);
	cbi(*s->ucsrb, s->rxcie);
	cbi(*s->ucsrb, s->udrie);

	/* clear any received data */
	s->rx_buffer->head = s->rx_buffer->tail;
}

unsigned int serial_avail(struct serial *s)
{
	return (unsigned int)(SERIAL_BUFFER_SIZE + s->rx_buffer->head -
			s->rx_buffer->tail) % SERIAL_BUFFER_SIZE;
}

int serial_peek(struct serial *s)
{
	if(s->rx_buffer->head == s->rx_buffer->tail) {
		return -1;
	} else {
		return s->rx_buffer->buffer[s->rx_buffer->tail];
	}
}

int serial_readchar(struct serial *s, char *c)
{
	/* if the head isn't ahead of the tail, we don't have any characters */
	if(s->rx_buffer->head == s->rx_buffer->tail) {
		return -1;
	}

	*c = s->rx_buffer->buffer[s->rx_buffer->tail];
	s->rx_buffer->tail = (unsigned int)(s->rx_buffer->tail + 1) %
		SERIAL_BUFFER_SIZE;
	return 0;
}


size_t serial_read(struct serial *s, void *buf, size_t count)
{
	size_t i;
	char *read = buf;
	char c;

	for(i = 0; i < count; ++i) {
		if((serial_readchar(s, &c)) < 0) {
			break;
		}
		*read = c;
		++read;
	}

	return i;
}

void serial_flush(struct serial *s)
{
	/*
	 * UDR is kept full while the buffer is not empty, so TXC triggers when
	 * EMPTY && SENT
	 */
	while(s->transmitting && !(*s->ucsra & _BV(TXC0)));
	s->transmitting = 0;
}

int serial_writechar(struct serial *s, uint8_t c)
{
	unsigned int i = (s->tx_buffer->head + 1) % SERIAL_BUFFER_SIZE;

	/*
	 * If the output buffer is full, there's nothing for it other than to 
	 * wait for the interrupt handler to empty it a bit
	 */
	while(i == s->tx_buffer->tail);


	s->tx_buffer->buffer[s->tx_buffer->head] = c;
	s->tx_buffer->head = i;

	sbi(*s->ucsrb, s->udrie);
	/*
	 * clear the TXC bit -- "can be cleared by writing a one to its bit
	 * location"
	 */
	s->transmitting = 1;
	sbi(*s->ucsra, TXC0);

	return 1;
}

int serial_write(struct serial *s, void const *buf, size_t nb)
{
	size_t i;
	for(i = 0; i < nb; ++i) {
		if(serial_writechar(s, ((char const *)buf)[i]) != 1) {
			return 0;
		}
	}
	return i;
}

#endif
