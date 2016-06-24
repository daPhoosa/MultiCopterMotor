/*
 *		MultiCopterMotor
 *		High speed update object using fixed point integer math to compute PWM input
 *		Assumes: 8 bit pwm and 488Hz update rate
 *		
 */

#include "MultiCopterMotor.h"


MultiCopterMotor::MultiCopterMotor(float mX, float mY, int16_t _spinDirection, int16_t _motorPin)
{
	motorPin = _motorPin;
	
	spinDirection = _spinDirection;
	
	if(abs(mX) + abs(mY) > 0.01f)
	{
		// normalize to unit vector
		float scale = 1.0f / sqrt(mX * mX + mY * mY);
		motorX = int(mX * scale * 32767.0f); // S0.15 fixed point number
		motorY = int(mY * scale * 32767.0f);
	}
	else
	{
		motorX = 0;	// this case would be used by a central lifting prop that does not contribute to active stability
		motorY = 0;	
	}
	
	analogWrite(motorPin, MCM_MOTOR_PULSE_MIN >> 3);  // use analogWrite to set pin and connect to pwm
}


void MultiCopterMotor::update(int16_t compX, int16_t compY, int16_t compYaw, int16_t throttle) // 20us
{
	// All arguments are expected to have units of integer microseconds
	
	int16_t pulseLength;
	
	pulseLength  = throttle;	// throttle component
	
	pulseLength += mul(compX << 1, motorX);	// X component -- S15.0 << 1 * S0.15 = S15.16 : mul returns the top S15
	pulseLength += mul(compY << 1, motorY);	// Y component
	
	if(spinDirection > 0)  // Yaw component
	{
		pulseLength += compYaw;	// positive spin
	}
	else
	{
		pulseLength -= compYaw; // negative spin
	}

	PWM_write(motorPin, constrain(pulseLength, MCM_MOTOR_PULSE_IDLE, MCM_MOTOR_PULSE_MAX) >> 3);		// constrain and divide by 8 to scale to 8bit PWM range and output to motor
}

void MultiCopterMotor::stop()
{
	PWM_write(motorPin, MCM_MOTOR_PULSE_MIN >> 3);
}

// **** Special Integer Multiplication ****
// signed 16 * signed 16 >> H16
// http://mekonik.wordpress.com/2009/03/18/arduino-avr-gcc-multiplication/
#define MultiS16X16toH16(intRes, intIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %A2 \n\t" \
"mov r27, r1 \n\t" \
"muls %B1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mulsu %B2, %A1 \n\t" \
"sbc %B0, r26 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mulsu %B1, %A2 \n\t" \
"sbc %B0, r26 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"a" (intIn1), \
"a" (intIn2) \
: \
"r26", "r27" \
)

// multiplication wrapper function
int16_t  __attribute__ ((noinline)) MultiCopterMotor::mul(int16_t a, int16_t b) {
	int16_t r;
	MultiS16X16toH16(r, a, b);
	return r;
}


void MultiCopterMotor::PWM_write(uint8_t pin, int val) // Borrowed from Arduino wiring_analog.c  -- remove setting pin mode to save time
{

	switch(digitalPinToTimer(pin))
	{
		// XXX fix needed for atmega8
		#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		case TIMER0A:
			OCR0 = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR0A) && defined(COM0A1)
		case TIMER0A:
			OCR0A = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR0A) && defined(COM0B1)
		case TIMER0B:
			OCR0B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:
			OCR1A = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:
			OCR1B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:
			OCR1C = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR2) && defined(COM21)
		case TIMER2:
			OCR2 = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR2A) && defined(COM2A1)
		case TIMER2A:
			OCR2A = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR2A) && defined(COM2B1)
		case TIMER2B:
			OCR2B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3A1)
		case TIMER3A:
			OCR3A = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3B1)
		case TIMER3B:
			OCR3B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3C1)
		case TIMER3C:
			OCR3C = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR4A)
		case TIMER4A:
			OCR4A = val;	// set pwm duty
			break;
		#endif
		
		#if defined(TCCR4A) && defined(COM4B1)
		case TIMER4B:
			OCR4B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR4A) && defined(COM4C1)
		case TIMER4C:
			OCR4C = val; // set pwm duty
			break;
		#endif
			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:				
			OCR4D = val;	// set pwm duty
			break;
		#endif

		#if defined(TCCR5A) && defined(COM5A1)
		case TIMER5A:
			OCR5A = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR5A) && defined(COM5B1)
		case TIMER5B:
			OCR5B = val; // set pwm duty
			break;
		#endif

		#if defined(TCCR5A) && defined(COM5C1)
		case TIMER5C:
			OCR5C = val; // set pwm duty
			break;
		#endif

		case NOT_ON_TIMER:
		default:
			digitalWrite(pin, LOW);

	}
}