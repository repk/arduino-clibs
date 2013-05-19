#ifndef _SPI_H_
#define _SPI_H_

#include <stdlib.h>
#include <stdint.h>
#include "spi_ring_buffer.h"

enum spi_mode {
	SPI_MASTER,
	SPI_SLAVE
};

enum spi_polarity {
	SPI_POL_HIGH,
	SPI_POL_LOW
};

enum spi_phase {
	SPI_PHA_SETUP,
	SPI_PHA_SAMPLE
};

enum spi_data_order {
	SPI_MSB_FIRST,
	SPI_LSB_FIRST
};

struct spi {
	struct spi_ring_buffer spi_rb;
	uint8_t spi_curr_spin;
#define NO_SPIN UINT8_MAX
	uint8_t spi_flags;
#define SPI_FLAG_MASTER	0x0001
#define SPI_FLAG_DORD	0x0010
#define SPI_FLAG_POL	0x0100
#define SPI_FLAG_PHA	0x1000
};

void spi_init(struct spi *s, enum spi_mode const  mode,
		enum spi_polarity const  pol, enum spi_phase const pha,
		enum spi_data_order const dorder);

int spi_begin(struct spi *s, unsigned long long const speedhz);

void spi_wait_txend(void);

int spi_rmslave(uint8_t const pin);
int spi_addslave(uint8_t const pin);

int spi_sendchar_async(struct spi *s, char const c, uint8_t const pin);
int spi_send(struct spi *s, void const *buf, size_t const count,
		uint8_t const pin);

char spi_sendchar_sync(struct spi *s, char const c, uint8_t const pin);
size_t spi_send_sync(struct spi *s, void const *src, void *dst,
		size_t const len, uint8_t const pin);

#endif
