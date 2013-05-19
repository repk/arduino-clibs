#ifndef _SPI_RING_BUFFER_
#define _SPI_RING_BUFFER_

#include <stdlib.h>
#include <stdint.h>

#if (RAMEND < 1000)
/* If your RAM is limited reduce the ring buffer size */
#define _SPI_BUFSZ 16
#else
#define _SPI_BUFSZ 64
#endif


struct spi_ring_buffer {
	unsigned char		srb_buf[_SPI_BUFSZ];
	volatile uint8_t	srb_head;
	volatile uint8_t	srb_tail;
};

#define _SRB_INIT(rb) do {						\
	(rb)->srb_head = 0;						\
	(rb)->srb_tail = 0;						\
} while(/*CONSTCOND*/0)

#define _SRB_EMPTY(rb) ((rb)->srb_head == (rb)->srb_tail)

int srb_push(struct spi_ring_buffer *r, unsigned char const c);
unsigned char srb_pop(struct spi_ring_buffer *r);

#endif
