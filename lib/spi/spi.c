#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>

#include <core/arduino.h>
#include <pins_arduino.h>

#include "spi.h"


struct _clk_tbl {
	uint8_t ct_div;
	uint8_t ct_spr0;
	uint8_t ct_spr1;
	uint8_t ct_spi2x;
};


/**
 * This table lists the different speed configurations
 */
PROGMEM static struct _clk_tbl const sck[] = {
	{1, 0, 0, 1},
	{2, 0, 0, 0},
	{3, 1, 0, 1},
	{4, 1, 0, 0},
	{5, 0, 1, 1},
	{6, 0, 1, 0},
	{7, 1, 1, 1},
	{8, 1, 1, 0},
};



/**
 * Initialize a spi device
 */
void spi_init(struct spi *s, enum spi_mode mode, enum spi_polarity pol,
		enum spi_phase pha, enum spi_dord dorder)
{
	uint8_t spcr = SPCR;

	s->spi_spin = NO_SPIN;
	spi_setflags(s, mode, pol, pha, dorder);

	/* SPI mode */
	if(mode == SMODE_SLAVE) {
		pinMode(SS, INPUT);
		spcr &= ~(1 << MSTR);
	} else {
		spcr |= (1 << MSTR);
	}

	/* Polarization and pahse */
	if(pol == SPOL_HIGH) {
		spcr &= ~(1 << CPOL);
	} else {
		spcr |= CPOL;
	}
	if(pha == SPHA_SAMPLE) {
		spcr &= ~(1 << CPHA);
	} else {
		spcr |= (1 << CPHA);
	}

	/* Data order */
	if(dorder == SDORD_MSB_FIRST) {
		spcr &= ~(1 << DORD);
	} else {
		spcr |= (1 << DORD);
	}

	/* Do not intercept interrupts */
	spcr &= ~(1 << SPIE);
	/* Enable spi */
	spcr |= (1 << SPE);

	SPCR = spcr;

	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);

}



/**
 * Set SPI speed
 */
int spi_begin(struct spi *s, unsigned long long speedhz)
{
	size_t i, nb = sizeof(sck)/sizeof(sck[0]);
	uint8_t f, spcr = SPCR, spsr = SPSR;
	(void) s;

	/* Find appropriate speed configuration variables */
	for(i = 0; i < nb; ++i) {
		f = pgm_read_byte(&sck[i].ct_div);
		if((F_CPU >> f) == (long long)(speedhz)) {
			break;
		}
	}

	/* Speed cannot be set as it is not handled by hardware */
	if(i == nb) {
		return -1;
	}

	/* Set speed */
	if(pgm_read_byte(&sck[i].ct_spr0) == 0) {
		spcr &= ~(1 << SPR0);
	} else {
		spcr |= (1 << SPR0);
	}

	if(pgm_read_byte(&sck[i].ct_spr1) == 0) {
		spcr &= ~(1 << SPR1);
	} else {
		spcr |= (1 << SPR1);
	}

	if(pgm_read_byte(&sck[i].ct_spi2x) == 0) {
		spsr &= ~(1 << SPI2X);
	} else {
		spsr |= (1 << SPI2X);
	}

	SPSR = spsr;
	SPCR = spcr;

	return 0;
}


/**
 * Ending SPI device
 */
void spi_end(struct spi *s)
{
	(void) s;
	SPCR &= ~(1 << SPE);
	s->spi_spin = NO_SPIN;
}


/**
 * Select a slave device
 */
void spi_slave_set(struct spi *s, uint8_t pin)
{
	if(spi_getmode(s) == SMODE_SLAVE)
		return;

	if(s->spi_spin == pin)
		return;

	if(s->spi_spin != NO_SPIN)
		spi_slave_unset(s);

	s->spi_spin = pin;
	digitalWrite(s->spi_spin, HIGH);
	pinMode(s->spi_spin, OUTPUT);

}

/**
 * Unselect a slave device
 */
void spi_slave_unset(struct spi *s)
{
	if(spi_getmode(s) == SMODE_SLAVE)
		return;

	if(s->spi_spin == NO_SPIN)
		return;

	digitalWrite(s->spi_spin, HIGH);
	s->spi_spin = NO_SPIN;
}


static inline char _spi_sendchar_sync(struct spi *s, char c)
{
	(void)s;
	SPDR = c;
	while((SPSR & (1 << SPIF)) == 0);
	return SPDR;
}


/**
 * Begin communication with the selected slave
 */
int spi_slave_start(struct spi *s)
{
	if(spi_getmode(s) == SMODE_SLAVE)
		return -1;

	if(s->spi_spin == NO_SPIN)
		return -1;

	digitalWrite(s->spi_spin, LOW);
	spi_slave_selected(s);

	return 0;
}


/**
 * Stop communication with the selected slave
 */
int spi_slave_stop(struct spi *s)
{
	if(spi_getmode(s) == SMODE_SLAVE)
		return -1;

	if(s->spi_spin == NO_SPIN)
		return -1;

	digitalWrite(s->spi_spin, HIGH);
	spi_slave_unselected(s);

	return 0;
}


/**
 * Send a single char (spi_slave_start and spi_slave_stop has to be called)
 *
 * This function is synchronous (wait till transmission completes)
 */
int spi_sendchar(struct spi *s, char c)
{
	if(!spi_slave_is_selected(s))
		return -1;

	_spi_sendchar_sync(s, c);
	return 0;
}


/**
 * Send a whole string (spi_slaveL_start() need to be called to select the
 * proper slave)
 *
 * This function is synchronous (wait till transmission completes)
 */
size_t spi_send(struct spi *s, void const *str, size_t len)
{
	uint8_t const *b = str;
	size_t i;

	if(!spi_slave_is_selected(s))
		return -1;

	for(i = 0; i < len; ++i) {
		_spi_sendchar_sync(s, b[i]);
	}

	return i;
}


/**
 * Select slave send a single char and unselect slave
 *
 * This function is synchronous (wait till transmission completes)
 */
int spi_slave_sendchar(struct spi *s, char c)
{
	if(s->spi_spin == NO_SPIN)
		return -1;

	digitalWrite(s->spi_spin, LOW);
	_spi_sendchar_sync(s, c);
	digitalWrite(s->spi_spin, HIGH);

	return 0;
}



/**
 * Select slave send a whole string then unselect slave
 *
 * This function is synchronous (wait till transmission completes)
 */
size_t spi_slave_send(struct spi *s, void const *str, size_t len)
{
	uint8_t const *b = str;
	size_t i;

	if(s->spi_spin == NO_SPIN)
		return -1;

	digitalWrite(s->spi_spin, LOW);
	for(i = 0; i < len; ++i) {
		_spi_sendchar_sync(s, b[i]);
	}
	digitalWrite(s->spi_spin, HIGH);

	return i;
}
