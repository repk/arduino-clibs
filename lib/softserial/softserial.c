#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <core/arduino.h>
#include <pins_arduino.h>

#include "softserial.h"

/**
 * These debug functions try to be as quicker as possible and as symetrical as
 * possible to not introduce anoying delays
 * With the help of an oscilloscope, the delay produced by a call to DEBUG_HIGH
 * or DEBUG_LOW is nearly 0.60685 microsec.
 */
#define DEBUG_SET	pinMode(13, OUTPUT)
#define DEBUG_HIGH	PORTB |= _BV(5)
#define DEBUG_LOW	PORTB &= ~_BV(5)


static struct softserial* volatile ss_active = NULL;

struct _ss_delay_table {
  unsigned long		dt_baud;
  unsigned short	dt_rx_delay_centering;
  unsigned short	dt_rx_delay_intrabit;
  unsigned short	dt_rx_delay_stopbit;
  unsigned short	dt_tx_delay;
};

#if F_CPU == 16000000
/*
 * With the help of an oscilloscope, the delay rate is 1 delay equals
 * approximately 0.43828 microsec.
 */

#define XMIT_START_ADJUSTMENT 5
PROGMEM static struct _ss_delay_table const dtbl[] = {
	{115200,	1,	17,	17,	12},
	{57600,		10,	37,	37,	33},
	{38400,		25,	57,	57,	54},
	{31250,		31,	70,	70,	68},
	{28800,		34,	77,	77,	74},
	{19200,		40,	112,	70,	114},
	{14400,		74,	156,	156,	153},
	{9600,		114,	236,	236,	233},
	{4800,		233,	474,	474,	471},
	{2400,		471,	950,	950,	947},
	{1200,		947,	1902,	1902,	1899},
	{600,		1902,	3804,	3804,	3800},
	{300,		3804,	7617,	7617,	7614},
};

#elif F_CPU == 8000000

#define XMIT_START_ADJUSTMENT 4
PROGMEM static struct _ss_delay_table const dtbl[] = {
	{115200,	1,	5,	5,	3},
	{57600,		1,	15,	15,	13},
	{38400,		2,	25,	26,	23},
	{31250,		7,	32,	33,	29},
	{28800,		11,	35,	35,	32},
	{19200,		20,	55,	55,	52},
	{14400,		30,	75,	75,	72},
	{9600,		50,	114,	114,	112},
	{4800,		110,	233,	233,	230},
	{2400,		229,	472,	472,	469},
	{1200,		467,	948,	948,	945},
	{600,		948,	1895,	1895,	1890},
	{300,		1895,	3805,	3805,	3802},
};

#elif F_CPU == 20000000

/**
 * 20MHz support courtesy of the good people at macegr.com.
 * Thanks, Garrett!
 */

#define XMIT_START_ADJUSTMENT 6
PROGMEM static struct _ss_delay_table const dtbl[] =
{
	{115200,	3,	21,	21,	18},
	{57600,		20,	43,	43,	41},
	{38400,		37,	73,	73,	70},
	{31250,		45,	89,	89,	88},
	{28800,		46,	98,	98,	95},
	{19200,		71,	148,	148,	145},
	{14400,		96,	197,	197,	194},
	{9600,		146,	297,	297,	294},
	{4800,		296,	595,	595,	592},
	{2400,		592,	1189,	1189,	1186},
	{1200,		1187,	2379,	2379,	2376},
	{600,		2379,	4759,	4759,	4755},
	{300,		4759,	9523,	9523,	9520},
};

#else
#error This version of softserial supports only 20, 16 and 8MHz processors
#endif

#define SS_RX_RD(s) ((*(s)->ss_rx_port_reg) & (s)->ss_rx_bitmask)
#define SS_TX_WR(s, p) do {						\
	if(p == LOW) {							\
		*s->ss_tx_port_reg &= ~s->ss_tx_bitmask;		\
	} else {							\
		*s->ss_tx_port_reg |= s->ss_tx_bitmask;			\
	}								\
} while(/*CONSTCOND*/0)

/* Wait for a specific delay */
static inline void softserial_wait(uint16_t delay)
{
	uint8_t tmp=0;

	asm volatile("sbiw    %0, 0x01 \n\t"
			"ldi %1, 0xFF \n\t"
			"cpi %A0, 0xFF \n\t"
			"cpc %B0, %1 \n\t"
			"brne .-10 \n\t"
			: "+r" (delay), "+a" (tmp)
			: "0" (delay));
}

static inline void softserial_isr_recv(struct softserial *s)
{
	struct ssring_buffer *rb;
	uint8_t i, ni;
	uint8_t d = 0;

	/**
	 * This test check if the isr is really for the rx pin or for another
	 * one
	 */
	if(((s->ss_flags & SS_MASK_INV_LOGIC) == 0)) {
		if(SS_RX_RD(s)) {
			return;
		}
	} else if (!SS_RX_RD(s)) {
		return;
	}

	/* Wait 1/2 bit width for centreing the sample */
	softserial_wait(s->ss_rx_delay_centering);

	/* Reach each 8 bits */
	for(i = 0x1; i; i <<= 1) {
		/* DEBUG_LOW; */
		/* Wait for next bit */
		softserial_wait(s->ss_rx_delay_intrabit);
		/* DEBUG_HIGH; */
		ni = ~i;
		if(SS_RX_RD(s)) {
			d |= i;
		} else {
			/**
			 * This is not really needed theoretically. But
			 * in order to balance the function timing I
			 * will add it anyway
			 *
			 * This is also why we used a intermediate ni
			 * variable
			 */
			d &= ni;
		}
	}

	/**
	 * Skip the stop bit, in fact we do not need to skip the entire stop bit
	 * and if we do so we can miss the start one. This is why the stop bit
	 * delay is compute to only wait enough time to skip the stop bit upper
	 * front.
	 *
	 * And if delay ar not accurate enough the pin change interrupt at start
	 * bit will recenter everything for us
	 */
	softserial_wait(s->ss_rx_delay_stopbit);

	if(s->ss_flags & SS_MASK_INV_LOGIC) {
		d = ~d;
	}

	rb = &s->ss_rx_rb;
	if((rb->rb_tail + 1) % _RB_BUFSZ != rb->rb_head) {
		rb->rb_buff[rb->rb_tail] = d;
		rb->rb_tail = (rb->rb_tail + 1) % _RB_BUFSZ;
	} else {
		s->ss_flags |= SS_MASK_OVERFLOW;
	}
}

static inline void softserial_isr_run(void)
{
	if(ss_active != NULL) {
		softserial_isr_recv(ss_active);
	}
}

/* ISR for all pin change interrupt */
#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
	softserial_isr_run();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect)
{
	softserial_isr_run();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect)
{
	softserial_isr_run();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect)
{
	softserial_isr_run();
}
#endif

void softserial_run_recv(void)
{
	softserial_isr_run();
}

static inline void softserial_set_tx(struct softserial *s, uint8_t p)
{
	uint8_t port;
	pinMode(p, OUTPUT);
	digitalWrite(p, HIGH);
	s->ss_tx_bitmask = digitalPinToBitMask(p);
	port = digitalPinToPort(p);
	s->ss_tx_port_reg = portOutputRegister(port);
}

static inline void softserial_set_rx(struct softserial *s, uint8_t p)
{
	uint8_t port;
	pinMode(p, INPUT);
	if((s->ss_flags & SS_MASK_INV_LOGIC) == 0) {
		digitalWrite(p, HIGH);
	}
	s->ss_rx_pin = p;
	s->ss_rx_bitmask = digitalPinToBitMask(p);
	port = digitalPinToPort(p);
	s->ss_rx_port_reg = portInputRegister(port);
}

static inline void _softserial_init(struct softserial *s, uint8_t rx_pin,
		uint8_t tx_pin, uint8_t inv_logic)
{
	s->ss_rx_delay_centering = 0;
	s->ss_rx_delay_intrabit = 0;
	s->ss_rx_delay_stopbit = 0;
	s->ss_tx_delay = 0;
	s->ss_flags = 0;
	if(inv_logic) {
		s->ss_flags |= SS_MASK_INV_LOGIC;
	}
	RING_BUFFER_INIT(&s->ss_rx_rb);
	softserial_set_rx(s, rx_pin);
	softserial_set_tx(s, tx_pin);
}

void softserial_init_logic(uint8_t rx_pin, uint8_t tx_pin, uint8_t inv_logic)
{
	_softserial_init(&sserial, rx_pin, tx_pin, inv_logic);
}

void softserial_init(uint8_t rx_pin, uint8_t tx_pin)
{
	_softserial_init(&sserial, rx_pin, tx_pin, 0);
}

static inline int softserial_listen(struct softserial *s)
{
	uint8_t oldSREG;
	if(s == ss_active) {
		return 0;
	}

	s->ss_flags &= ~SS_MASK_OVERFLOW;
	oldSREG = SREG;
	cli();
	RING_BUFFER_INIT(&s->ss_rx_rb);
	ss_active = s;
	SREG = oldSREG;
	return 0;
}

int softserial_begin(struct softserial *s, unsigned long baud)
{
	unsigned long b;
	unsigned int i;
	int8_t rxp = s->ss_rx_pin;

	s->ss_rx_delay_centering = 0;
	DEBUG_SET;

	for(i = 0; i < sizeof(dtbl)/sizeof(dtbl[0]); ++i) {
		b = pgm_read_dword(&dtbl[i].dt_baud);
		if(b == baud) {
			s->ss_rx_delay_centering =
				pgm_read_word(&dtbl[i].dt_rx_delay_centering);
			s->ss_rx_delay_intrabit =
				pgm_read_word(&dtbl[i].dt_rx_delay_intrabit);
			s->ss_rx_delay_stopbit =
				pgm_read_word(&dtbl[i].dt_rx_delay_stopbit);
			s->ss_tx_delay = pgm_read_word(&dtbl[i].dt_tx_delay);
			break;
		}
	}
	if(s->ss_rx_delay_centering == 0) {
		return -1;
	}

	/* digitalPinToPCICR -> register for pin change interrupt */
	if(digitalPinToPCICR(rxp) == NULL) {
		return -2;
	}

	*digitalPinToPCICR(rxp) |= _BV(digitalPinToPCICRbit(rxp));
	*digitalPinToPCMSK(rxp) |= _BV(digitalPinToPCMSKbit(rxp));
	softserial_wait(s->ss_tx_delay);

	return softserial_listen(s);
}

int softserial_end(struct softserial *s)
{
	uint8_t p = s->ss_rx_pin;
	if(digitalPinToPCMSK(p)) {
		*digitalPinToPCMSK(p) &= ~_BV(digitalPinToPCMSKbit(p));
	}
	return 0;
}

int softserial_readchar(struct softserial *s, char *c)
{
	struct ssring_buffer *rb;
	if(ss_active != s) {
		return -1;
	}

	rb = &s->ss_rx_rb;
	if(rb->rb_head == rb->rb_tail) {
		return -2;
	}
	*c = rb->rb_buff[rb->rb_head];
	rb->rb_head = (rb->rb_head + 1) % _RB_BUFSZ;

	return 0;
}

size_t softserial_read(struct softserial *s, void *buf, size_t count)
{
	char *read = buf;
	size_t i;
	char c;

	for(i = 0; i < count; ++i) {
		if(softserial_readchar(s, &c) < 0) {
			break;
		}
		*read = c;
		++read;
	}

	return i;
}

int softserial_writechar(struct softserial *s, uint8_t c)
{
	uint8_t i;
	uint8_t oldSREG;

	if(s->ss_tx_delay == 0) {
		return -1;
	}

	oldSREG = SREG;
	cli();
	if(s->ss_flags & SS_MASK_INV_LOGIC) {
		/* Send the start bit */
		SS_TX_WR(s, HIGH);
		softserial_wait(s->ss_tx_delay + XMIT_START_ADJUSTMENT);

		for(i = 0x1; i; i <<= 1) {
			if(c & i) {
				SS_TX_WR(s, LOW);
			} else {
				SS_TX_WR(s, HIGH);
			}
			softserial_wait(s->ss_tx_delay);
		}
		/* Stop bit */
		SS_TX_WR(s, LOW);
	} else {
		/* Send the start bit */
		SS_TX_WR(s, LOW);
		softserial_wait(s->ss_tx_delay + XMIT_START_ADJUSTMENT);

		for(i = 0x1; i; i <<= 1) {
			if(c & i) {
				SS_TX_WR(s, HIGH);
			} else {
				SS_TX_WR(s, LOW);
			}
			softserial_wait(s->ss_tx_delay);
		}
		/* Stop bit */
		SS_TX_WR(s, HIGH);
	}
	SREG = oldSREG;
	softserial_wait(s->ss_tx_delay);

	return 0;
}

int softserial_write(struct softserial *s, void const *buf, size_t nb)
{
	size_t i;
	for(i = 0; i < nb; ++i) {
		if(softserial_writechar(s, ((char const *)buf)[i]) < 0) {
			return i;
		}
	}
	return i;
}

unsigned int softserial_avail(struct softserial *s)
{
	return (unsigned int)(_RB_BUFSZ + s->ss_rx_rb.rb_head -
			s->ss_rx_rb.rb_tail) % _RB_BUFSZ;
}
