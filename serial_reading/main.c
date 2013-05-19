/**
 * Use serial usb to read some number in the computer
 */

#include <core/arduino.h>
#include <serial/serial.h>


int main(void)
{
	char s[] = "Hello World\r\n";
	init();
	serial_init();

	serial_begin(&serial[0], 9600);

	while(1) {
		serial_write(&serial[0], s, sizeof(s));
		delay(1000);
	}
	return 0;
}
