/**
 * Use serial usb to read some number in the computer
 */

#include <core/arduino.h>
#include <softserial/softserial.h>
#include <avr/sleep.h>

#define BUFSZ 511


int main(void)
{
	char buf[BUFSZ + 1];
	int i;

	init();

	softserial_init(2, 3);
	softserial_begin(&sserial, 19200);
	softserial_write(&sserial, "\r\n", sizeof("\r\n"));

	while(1) {
		i = softserial_read(&sserial, buf, BUFSZ);
		if(i > 0) {
			softserial_write(&sserial, buf, i);

		}
		delay(200);
	}
	return 0;
}

