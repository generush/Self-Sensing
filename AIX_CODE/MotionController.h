// MotionController.h
// Marc Petit
// 04/02/15 
#pragma once
#include "constants.h"
#include <math.h>

float norm_angle(float angle);

class MotionController 
{
	private:
		// Gains
		float 	Kipa, Kpa, Kva;
		// State Variables
		float 	omega_com, omega_k;
		float 	Tem_com;
		// Auxiliary Variables
		float M_ip, M_p, M_v, omega_err; 
		// Member functions
		void  	run();
	public:
		MotionController();
		void  	update(const float new_omega_com, const float new_omega_k);
		void 	clear();
		float 	get_Tem_com() const;
		float	get_gains(const int& i) const;
};
