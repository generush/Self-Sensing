#pragma once
#ifdef VDSP
	#include <shared/nop.h>
#endif
template<class T>
inline
void init_pwm (T& PWM, const float pwm_frequency, const float pwm_deadtime)
{
	#ifdef VDSP
		// PWM set Zero state: all even PWM output channels are inverted
		// PWM value zero means: BOT switch ON, TOP switch OFF
		PWM.ZEROSTATE = 0x55;
		
		// disable forcing of PWM channels and set force value to 0
		PWM.HOLDENABLE = 0x00;
		PWM.HOLDENABLE = 0x00;

		
		// select PWM center aligned generation mode
		PWM.CENTER.set();

		// set PWM base frequency
		PWM.FREQ = pwm_frequency;
		
		// declare pairs of switching signals and setup deadtimes 
		// PAIR0 -> 0:BOT, 1:TOP  
		// PAIR1 -> 2:BOT, 3:TOP ...
		PWM.PAIR0.set_deadtime(pwm_deadtime);
		PWM.PAIR1.set_deadtime(pwm_deadtime);
		PWM.PAIR2.set_deadtime(pwm_deadtime);
		PWM.PAIR3.set_deadtime(pwm_deadtime);
		
		// init PWM pairs output signal
		PWM.PAIR0 = PWM.PAIR1 = PWM.PAIR2 = 0.5f;

		// declare independant PWM output signal for chopper
		PWM.CH6 = 0.0f;
		PWM.CH7 = 0.0f;

		// select PWM generation on all channels 0..6		
		PWM.ENABLE = 0xFF;

	#else

		// Set Zero State of Outputs (inverted bit)
		pwm_zero_state  (PWM.CH1,false);	
		pwm_zero_state  (PWM.CH3,false);	
		pwm_zero_state  (PWM.CH5,false);	
		pwm_zero_state  (PWM.CH7,false);	
		pwm_zero_state  (PWM.CH0,true );	
		pwm_zero_state  (PWM.CH2,true );	
		pwm_zero_state  (PWM.CH4,true );	
		pwm_zero_state  (PWM.CH6,true );	
	
		// disable all Force enable states
		pwm_force_enable(PWM.CH0,false);
		pwm_force_enable(PWM.CH1,false);
		pwm_force_enable(PWM.CH2,false);
		pwm_force_enable(PWM.CH3,false);
		pwm_force_enable(PWM.CH4,false);
		pwm_force_enable(PWM.CH5,false);
		pwm_force_enable(PWM.CH6,false);
		pwm_force_enable(PWM.CH7,false);
	
		// set all Force values to false
		pwm_force_value (PWM.CH0,false);
		pwm_force_value (PWM.CH1,false);
		pwm_force_value (PWM.CH2,false);
		pwm_force_value (PWM.CH3,false);
		pwm_force_value (PWM.CH4,false);
		pwm_force_value (PWM.CH5,false);
		pwm_force_value (PWM.CH6,false);
		pwm_force_value (PWM.CH7,false);
		
		// configure PWM Center mode and switching Frequency
		pwm_set_frequency         (PWM, pwm_frequency, true);

		// init Deadtime for combined channels
		pwm_set_combined_deadtime (PWM.PAIR0, pwm_deadtime);
		pwm_set_combined_deadtime (PWM.PAIR1, pwm_deadtime);
		pwm_set_combined_deadtime (PWM.PAIR2, pwm_deadtime);
		pwm_set_combined_deadtime (PWM.PAIR3, pwm_deadtime);
		
		// set all combined Dutycycles to 0.5	
		pwm_set_combined_dutycycle(PWM.PAIR0, 0.5);
		pwm_set_combined_dutycycle(PWM.PAIR1, 0.5);
		pwm_set_combined_dutycycle(PWM.PAIR2, 0.5);
		pwm_set_combined_dutycycle(PWM.PAIR3, 0.5);	

		// enable PWM function for all IGBT channels
		pwm_enable (PWM.CH0);
		pwm_enable (PWM.CH1);
		pwm_enable (PWM.CH2);
		pwm_enable (PWM.CH3);
		pwm_enable (PWM.CH4);
		pwm_enable (PWM.CH5);
		pwm_enable (PWM.CH6);
		
		// disable Relais output
		pwm_disable(PWM.CH7);
	#endif
}

