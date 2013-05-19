#include <core/arduino.h>



int main(void)
{
	int pin = 13;

	init();

	/* assumes the LED is connected to pin 13 on the arduino uno */
	pinMode(pin, OUTPUT);

	while(1) {
		digitalWrite(pin, HIGH);  /* turn the LED on */
		delay(1000);              /* wait for a second */
		digitalWrite(pin, LOW);   /* turn the LED off */
		delay(1000);              /* wait for a second */
	}

	return 0;
}
