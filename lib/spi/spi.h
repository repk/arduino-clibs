#ifndef _SPI_H_
#define _SPI_H_

#include <stdlib.h>
#include <stdint.h>

enum spi_mode {
	SMODE_MASTER,
	SMODE_SLAVE
};


enum spi_polarity {
	SPOL_HIGH,
	SPOL_LOW
};


enum spi_phase {
	SPHA_SETUP,
	SPHA_SAMPLE,
};

enum spi_dord {
	SDORD_MSB_FIRST,
	SDORD_LSB_FIRST
};


struct spi {
	uint8_t spi_spin; /* Pin connected to slave (SS) */
	uint8_t spi_flags;
};

#define NO_SPIN UINT8_MAX

#define SFLAG_MASTER	1
#define SFLAG_DORD	2
#define SFLAG_POL	4
#define SFLAG_PHA	8
#define SFLAG_SS	16


static inline void spi_setflags(struct spi *s, enum spi_mode mode,
		enum spi_polarity pol, enum spi_phase pha, enum spi_dord dorder)
{
	int fl = 0;
	if(mode == SMODE_MASTER) {
		fl |= SFLAG_MASTER;
	}
	if(pol == SPOL_LOW) {
		fl |= SFLAG_POL;
	}
	if(pha == SPHA_SAMPLE) {
		fl |= SFLAG_PHA;
	}
	if(dorder == SDORD_LSB_FIRST) {
		fl |= SFLAG_DORD;
	}
	s->spi_flags = fl;
}


static inline enum spi_mode spi_getmode(struct spi const *s)
{
	if(s->spi_flags & SFLAG_MASTER) {
		return SMODE_MASTER;
	}
	return SMODE_SLAVE;
}


static inline enum spi_polarity spi_getpolarity(struct spi const *s)
{
	if(s->spi_flags & SFLAG_POL) {
		return SPOL_LOW;
	}
	return SPOL_HIGH;
}


static inline enum spi_phase spi_getphase(struct spi const *s)
{
	if(s->spi_flags & SFLAG_PHA) {
		return SPHA_SAMPLE;
	}
	return SPHA_SETUP;
}


static inline enum spi_dord spi_getdata_order(struct spi const *s)
{
	if(s->spi_flags & SFLAG_DORD) {
		return SDORD_LSB_FIRST;
	}
	return SDORD_MSB_FIRST;
}


static inline int spi_slave_is_selected(struct spi const *s)
{
	return (s->spi_flags & SFLAG_SS);
}


static inline void spi_slave_selected(struct spi *s)
{
	s->spi_flags |= SFLAG_SS;
}


static inline void spi_slave_unselected(struct spi *s)
{
	s->spi_flags &= ~(SFLAG_SS);
}


/**
 * Initialize a spi device
 */
void spi_init(struct spi *s, enum spi_mode mode, enum spi_polarity pol,
		enum spi_phase pha, enum spi_dord dorder);


/**
 * Set SPI speed
 */
int spi_begin(struct spi *s, unsigned long long speedhz);
/**
 * Ending SPI device
 */
void spi_end(struct spi *s);


/**
 * Select a slave device
 */
void spi_slave_set(struct spi *s, uint8_t pin);
/**
 * Unselect a slave device
 */
void spi_slave_unset(struct spi *s);


/**
 * Begin communication with the selected slave
 */
int spi_slave_start(struct spi *s);
/**
 * Stop communication with the selected slave
 */
int spi_slave_stop(struct spi *s);


/**
 * Send a single char (spi_slave_start and spi_slave_stop has to be called)
 *
 * This function is synchronous (wait till transmission completes)
 */
int spi_sendchar(struct spi *s, char c);
/**
 * Send a whole string (spi_slaveL_start() need to be called to select the
 * proper slave)
 *
 * This function is synchronous (wait till transmission completes)
 */
size_t spi_send(struct spi *s, void const *str, size_t len);


/**
 * Select slave send a single char and unselect slave
 *
 * This function is synchronous (wait till transmission completes)
 */
int spi_slave_sendchar(struct spi *s, char c);
/**
 * Select slave send a whole string then unselect slave
 *
 * This function is synchronous (wait till transmission completes)
 */
size_t spi_slave_send(struct spi *s, void const *str, size_t len);

#endif
