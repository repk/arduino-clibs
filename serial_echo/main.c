/**
 * Use serial usb to read some number in the computer
 */
#include <avr/sleep.h>
#include <core/arduino.h>
#include <serial/serial.h>

#define BUFSZ 511


int main(void)
{
	int i;
	char buf[BUFSZ + 1];

	init();
	serial_init();

	serial_begin(&serial[0], 19200);

	serial_write(&serial[0], "Hello world\r\n", sizeof("Hello world\r\n"));

	while(1) {
		i = serial_read(&serial[0], buf, BUFSZ);
		if(i > 0) {
			serial_write(&serial[0], buf, i);
		}
		delay(1000);
	}
	return 0;
}

