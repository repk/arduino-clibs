/**
 * Use serial usb to read some number in the computer
 */

#include <core/arduino.h>
#include <serial/serial.h>
#include <softserial/softserial.h>

#define BUFSZ 511


int main(void)
{
	int i;
	char buf[BUFSZ + 1];

	init();
	serial_init();
	softserial_init(2, 3);

	serial_begin(&serial[0], 19200);
	softserial_begin(&sserial, 19200);
	serial_write(&serial[0], "\r\n", 2);
	while(1) {
		i = serial_read(&serial[0], buf, BUFSZ);
		if(i > 0) {
			softserial_write(&sserial, buf, i);
		}
		i = softserial_read(&sserial, buf, BUFSZ);
		if(i > 0) {
			serial_write(&serial[0], buf, i);
		}
	}
	return 0;
}

