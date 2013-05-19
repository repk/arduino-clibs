#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>

#include <core/arduino.h>

#include "spi.h"

static struct spi *spi_current = NULL;

struct _dly_tbl {
	uint8_t dt_div;
	uint8_t dt_spr0;
	uint8_t dt_spr1;
	uint8_t dt_spi2x;
};

PROGMEM static struct _dly_tbl const _dly_div[] = {
	{1, 0, 0, 1},
	{2, 0, 0, 0},
	{3, 0, 1, 1},
	{4, 0, 1, 0},
	{5, 1, 0, 1},
	{6, 1, 0, 0},
	{7, 1, 1, 1},
	{8, 1, 1, 0},
};

static inline void spi_setflags(struct spi *s, enum spi_mode const mode,
		enum spi_polarity const  pol, enum spi_phase const pha,
		enum spi_data_order const dorder)
{
	int fl = 0;
	if(mode == SPI_SLAVE) {
		fl |= SPI_FLAG_MASTER;
	}
	if(pol == SPI_POL_LOW) {
		fl |= SPI_FLAG_POL;
	}
	if(pha == SPI_PHA_SAMPLE) {
		fl |= SPI_FLAG_PHA;
	}
	if(dorder == SPI_LSB_FIRST) {
		fl |= SPI_FLAG_DORD;
	}
	s->spi_flags = fl;
}

static inline enum spi_mode spi_getmode(struct spi const *s)
{
	if(s->spi_flags & SPI_FLAG_MASTER) {
		return SPI_SLAVE;
	}
	return SPI_MASTER;
}

static inline enum spi_polarity spi_getpolarity(struct spi const *s)
{
	if(s->spi_flags & SPI_FLAG_POL) {
		return SPI_POL_LOW;
	}
	return SPI_POL_HIGH;
}

static inline enum spi_phase spi_getphase(struct spi const *s)
{
	if(s->spi_flags & SPI_FLAG_PHA) {
		return SPI_PHA_SAMPLE;
	}
	return SPI_PHA_SETUP;
}

static inline enum spi_data_order spi_getdata_order(struct spi const *s)
{
	if(s->spi_flags & SPI_FLAG_DORD) {
		return SPI_LSB_FIRST;
	}
	return SPI_MSB_FIRST;
}

void spi_init(struct spi *s, enum spi_mode const  mode,
		enum spi_polarity const  pol, enum spi_phase const pha,
		enum spi_data_order const dorder)
{
	_SRB_INIT(&s->spi_rb);
	s->spi_curr_spin = NO_SPIN;
	spi_setflags(s, mode, pol, pha, dorder);
}


int spi_begin(struct spi *s, unsigned long long const speedhz)
{
	size_t i, nb = sizeof(_dly_div)/sizeof(_dly_div[0]);
	uint8_t f;

	/* Find appropriate speed configuration variables */
	for(i = 0; i < nb; ++i) {
		f = pgm_read_byte(&_dly_div[i].dt_div);
		if(F_CPU == (speedhz << f)) {
			break;
		}
	}
	/* Speed cannot be set as it is not handled by hardware */
	if(i == nb) {
		return -1;
	}

	/* Set speed */
	if(pgm_read_byte(&_dly_div[i].dt_spr0) == 0) {
		SPCR &= ~(1 << SPR0);
	} else {
		SPCR |= (1 << SPR0);
	}

	if(pgm_read_byte(&_dly_div[i].dt_spr1) == 0) {
		SPCR &= ~(1 << SPR1);
	} else {
		SPCR |= (1 << SPR1);
	}

	if(pgm_read_byte(&_dly_div[i].dt_spi2x) == 0) {
		SPSR &= ~(1 << SPI2X);
	} else {
		SPSR |= (1 << SPI2X);
	}

	/* Polarity and Phase */
	if(spi_getpolarity(s) == SPI_POL_HIGH) {
		SPCR &= ~(1 << CPOL);
	} else {
		SPCR |= CPOL;
	}
	if(spi_getphase(s) == SPI_PHA_SAMPLE) {
		SPCR &= ~(1 << CPHA);
	} else {
		SPCR |= (1 << CPHA);
	}

	/* Data order */
	if(spi_getdata_order(s) == SPI_LSB_FIRST) {
		SPCR &= ~(1 << DORD);
	} else {
		SPCR |= (1 << DORD);
	}

	/* SPI mode */
	if(spi_getmode(s) == SPI_SLAVE) {
		SPCR &= ~(1 << MSTR);
	} else {
		SPCR |= (1 << MSTR);
	}

	/* Do not intercept interrupts */
	SPCR &= ~(1 << SPIE);
	/* Enable spi */
	SPCR |= (1 << SPE);

/* XXX To Remove */
	pinMode(9, OUTPUT);
	return 0;
}


ISR(SPI_STC_vect)
{
	struct spi *s = spi_current;
	uint8_t c;
	PORTB |= 1 << 1;
	if(s == NULL || s->spi_curr_spin == NO_SPIN) {
		return;
	}
	if(_SRB_EMPTY(&s->spi_rb)) {
		digitalWrite(s->spi_curr_spin, HIGH);
		/* Do not intercept interrupts */
		SPCR &= ~(1 << SPIE);
		s->spi_curr_spin = NO_SPIN;
		spi_current = NULL;
	} else {
		c = srb_pop(&s->spi_rb);
		SPDR = c;
	}
/*XXX to be removed */
	PORTB &= ~(1 << 1);
}

void spi_wait_txend(void)
{
	while(spi_current != NULL);
}

int spi_addslave(uint8_t const pin)
{
	/* TODO Check if pin can be a SS pin */
	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH);
	return 0;
}

int spi_rmslave(uint8_t const pin)
{
	if(spi_current != NULL && spi_current->spi_curr_spin == pin) {
		spi_wait_txend();
	}
	digitalWrite(pin, LOW);
	return 0;
}

int spi_sendchar_async(struct spi *s, char const c, uint8_t const pin)
{
	int ret = 0;
	uint8_t first = 0;
	uint8_t OLDSREG = SREG;
	cli();

	if(spi_current == NULL) {
		spi_current = s;
		first = 1;
	} else if(spi_current != s) {
		SREG = OLDSREG;
		spi_wait_txend();
		cli();
		spi_current = s;
		first = 1;
	}

	if(s->spi_curr_spin == NO_SPIN) {
		s->spi_curr_spin = pin;
		digitalWrite(pin, LOW);
		first = 1;
	} else if(s->spi_curr_spin != pin) {
		SREG = OLDSREG;
		spi_wait_txend();
		cli();
		s->spi_curr_spin = pin;
		digitalWrite(pin, LOW);
		first = 1;
	}

	if(first) {
		SPCR |= (1 << SPIE);
		SPDR = c;
	} else {
		ret = srb_push(&s->spi_rb, c);
	}
	SREG = OLDSREG;
	return ret;
}

int spi_send(struct spi *s, void const *buf, size_t const count,
		uint8_t const pin)
{
	uint8_t const *b = buf;
	size_t i;

	for(i = 0; i < count; ++i) {
		if(spi_sendchar_async(s, b[i], pin) != 0) {
			break;
		}
	}

	return i;
}

static inline char _spi_sendchar_sync(char const c)
{
	SPDR = c;
	PORTB |= 1 << 1;
	while((SPSR & SPIF) == 0);
	PORTB &= ~(1 << 1);
	return SPDR;
}

char spi_sendchar_sync(struct spi *s, char const c, uint8_t const pin)
{
	char r;
	spi_wait_txend();
	if(s->spi_curr_spin != pin) {
		s->spi_curr_spin = pin;
		digitalWrite(pin, LOW);
	}
	r = _spi_sendchar_sync(c);
	digitalWrite(pin, HIGH);
	s->spi_curr_spin = NO_SPIN;
	return r;
}

size_t spi_send_sync(struct spi *s, void const *src, void *dst,
		size_t const len, uint8_t const pin)
{
	uint8_t const *b = src;
	uint8_t *d = dst;
	size_t i;
	PORTB |= 1 << 1;
	spi_wait_txend();
	PORTB &= ~(1 << 1);
	if(s->spi_curr_spin != pin) {
		s->spi_curr_spin = pin;
		digitalWrite(pin, LOW);
	}
	for(i = 0; i < len; ++i) {
		d[i] = _spi_sendchar_sync(b[i]);
	}
	digitalWrite(pin, HIGH);
	s->spi_curr_spin = NO_SPIN;
	return i;
}
