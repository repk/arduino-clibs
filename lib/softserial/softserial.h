#ifndef __SOFTSERIAL_H__
#define __SOFTSERIAL_H__

#define _RB_BUFSZ 64
struct ssring_buffer {
	char			rb_buff[_RB_BUFSZ];
	volatile uint8_t	rb_head;
	volatile uint8_t	rb_tail;
};

#define RING_BUFFER_INIT(rb) do {					\
	(rb)->rb_head = 0;						\
	(rb)->rb_tail = 0;						\
} while(/*CONSTCOND*/0)

struct softserial {
	struct ssring_buffer ss_rx_rb;
	volatile uint8_t *ss_rx_port_reg;
	volatile uint8_t *ss_tx_port_reg;

	uint16_t ss_rx_delay_centering;
	uint16_t ss_rx_delay_intrabit;
	uint16_t ss_rx_delay_stopbit;
	uint16_t ss_tx_delay;

	uint8_t ss_rx_pin;
	uint8_t ss_rx_bitmask;
	uint8_t ss_tx_bitmask;
	uint8_t ss_flags;
	#define SS_MASK_OVERFLOW	0x01
	#define SS_MASK_INV_LOGIC	0x10
};

struct softserial sserial;
void softserial_run_recv(void);
void softserial_init_logic(uint8_t rx_pin, uint8_t tx_pin, uint8_t inv_logic);
void softserial_init(uint8_t rx_pin, uint8_t tx_pin);
int softserial_begin(struct softserial *s, unsigned long baud);
int softserial_end(struct softserial *s);
int softserial_readchar(struct softserial *s, char *c);
size_t softserial_read(struct softserial *s, void *buf, size_t count);
int softserial_writechar(struct softserial *s, uint8_t c);
int softserial_write(struct softserial *s, void const *buf, size_t nb);
unsigned int softserial_avail(struct softserial *s);


#endif
