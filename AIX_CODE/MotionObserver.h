// MotionObserver.h
// Marc Petit
// 03/28/15 
#pragma once
#include "constants.h"
extern const int motor_pos;
float norm_angle(float angle);

class MotionObserver 
{
	private:
		// Gains
		float 	Kso, Kiso, bso;
		// State Variables
		float 	theta_k, theta_km1, theta_hat_k, theta_hat_kp1;
		float   del_theta_kp1, del_theta_err;
		float 	omega_hat_k, omega_hat_kp1;
		float 	Tem_com;
		float	T_d_hat; 
		// Auxiliary Variables
		float M_d_o, M_p_o, M_i_o, T_hat; 
		// Member functions
		void  	run();
		
	public:
		MotionObserver();
		void  	update(const float theta_k, const float Tem_com);
		void 	clear();
		float 	get_theta_kp1() const;
		float 	get_omega_kp1() const;
		float 	get_omega_left_kp1() const {return omega_hat_kp1;} 
		float	get_gains(const int& i) const;
};
