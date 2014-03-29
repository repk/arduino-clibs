#include <core/arduino.h>
#include <spi/spi.h>
#include <pins_arduino.h>



int main(void)
{
	struct spi s;

	init();
	spi_init(&s, SMODE_MASTER, SPOL_HIGH, SPHA_SAMPLE, SDORD_MSB_FIRST);
	spi_begin(&s, 8000000);

	spi_slave_select(&s, SS);

	spi_send_sync(&s, "Hello, World\n", sizeof("Hello, World\n") - 1);

	while(1);

	return 0;
}
