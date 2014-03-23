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
#define NO_SPIN UINT8_MAX
	uint8_t spi_spin; /* Pin connected to slave (SS) */
	uint8_t spi_flags;
#define SFLAG_MASTER	1
#define SFLAG_DORD	2
#define SFLAG_POL	4
#define SFLAG_PHA	8
};

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

void spi_init(struct spi *s, enum spi_mode mode, enum spi_polarity pol,
		enum spi_phase pha, enum spi_dord dorder);

int spi_begin(struct spi *s, unsigned long long speedhz);
void spi_end(struct spi *s);

void spi_select_slave(struct spi *s, uint8_t pin);
void spi_unselect_slave(struct spi *s);

int spi_sendchar_sync(struct spi *s, char const c);
size_t spi_send_sync(struct spi *s, void const *str, size_t const len);

#endif
