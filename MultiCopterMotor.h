/*
 *		MultiCopterMotor.h
 */
 
#ifndef MultiCopterMotor_h

	#define MultiCopterMotor_h

	#include "arduino.h"

	#define MCM_MOTOR_PULSE_MIN  1000
	#define MCM_MOTOR_PULSE_MAX  2000
	#define MCM_MOTOR_PULSE_IDLE 1100


	class MultiCopterMotor
	{
		public:
		
			MultiCopterMotor(float mX, float mY, int16_t _spinDirection, int16_t _motorPin);
			
			void update(int16_t compX, int16_t compY, int16_t compYaw, int16_t throttle);
			
			void stop();

			
		private:

			int16_t mul(int16_t a, int16_t b); // special multiplication function
			
			void PWM_write(uint8_t pin, int val);
			
			int16_t motorPin;
			int16_t spinDirection;
			int16_t motorX, motorY;	

	};


#endif