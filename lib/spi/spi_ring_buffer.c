#include "spi_ring_buffer.h"

#define BUG_ON(s)

int srb_push(struct spi_ring_buffer *r, unsigned char const c)
{
	size_t i = (r->srb_tail + 1) % _SPI_BUFSZ;
	if(i == r->srb_head) {
		return -1;
	}
	r->srb_buf[i] = c;
	r->srb_tail = i;
	return 0;
}

unsigned char srb_pop(struct spi_ring_buffer *r)
{
	unsigned char c;
	BUG_ON(_SRB_EMPTY(r));
	c = r->srb_buf[r->srb_head];
	r->srb_head = (r->srb_head + 1) % _SPI_BUFSZ;
	return c;
}
