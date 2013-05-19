/**
 * Use serial usb to read some number in the computer
 */

#include <core/arduino.h>
#include <serial/serial.h>


int main(void)
{
	char sms[] = "Voila un sms qui provient du futur. Signe un SIM900 et un arduino qui vous veulent du bien !\r\n";
	init();
	serial_init();

	serial_begin(&serial[0], 19200);
	serial_write(&serial[0], "\r\n", 2);
	delay(1000);
	serial_write(&serial[0], "AT+CMGF=1\r\n", sizeof("AT+CMGF=1\r\n"));
	delay(1000);

	/* Put your num right here */
	serial_write(&serial[0], "AT+CMGS=\"+336666666\"\r\n",
			sizeof("AT+CMGS=\"+336666666\"\r\n"));
	delay(1000);
	serial_write(&serial[0], sms, sizeof(sms));
	delay(1000);
	serial_writechar(&serial[0], 26);

	while(1) {
		delay(1000);
	}
	return 0;
}

