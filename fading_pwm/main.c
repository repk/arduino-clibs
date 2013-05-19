/**
 * analogWrite uses 8-bits pulse width modulation (PWM), turning a digital pin
 * on and off very quickly, to create a fading effect
 */

#include <core/arduino.h>


int main(void)
{
	unsigned char pwmpin = 9;
	unsigned char fadestep = 5;
	unsigned char brightness = 0;

	init();

	pinMode(pwmpin, OUTPUT);

	while(1) {
		analogWrite(pwmpin, brightness);

		brightness += fadestep;
		if(brightness == 0 || brightness == 255) {
			fadestep = -fadestep;
		}

		delay(30);
	}

	return 0;
}
