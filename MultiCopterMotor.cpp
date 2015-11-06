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
	
	int16_t spinDirection = _spinDirection;
	
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
	
	pwm = MCM_MOTOR_PULSE_MIN >> 3;
	
}


void MultiCopterMotor::update(int16_t compX, int16_t compY, int16_t compYaw, int16_t throttle) // 20us
{
	// All arguments are expected to have units of integer microseconds
	
	int16_t pulseLength;
	uint8_t pwm;
	
	pulseLength  = mul(compX << 1, motorX);	// X component -- S15.0 << 1 * S0.15 = S15.16 : mul returns the top S15
	pulseLength += mul(compY << 1, motorY);	// Y component
	pulseLength += compYaw * spinDirection;	// Yaw component
	pulseLength += throttle;				// throttle component
	
	pwm = constrain(pulseLength, MCM_MOTOR_PULSE_MIN, MCM_MOTOR_PULSE_MAX) >> 3;	// constrain and divide by 8 to scale to 8bit PWM range

	analogWrite(motorPin, pwm);
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