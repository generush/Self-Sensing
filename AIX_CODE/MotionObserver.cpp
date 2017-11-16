// MotionObserver.cpp
// Marc Petit
// 03/28/15 
#include <math.h>

# include "MotionObserver.h"

/*
	Input:
*/
MotionObserver::MotionObserver() 
{
	float a		=	expf(-C_2PI* MOBS_BW/100.f*C_SAMPLE_TIME); // IPos
	float b		=   expf(-C_2PI* MOBS_BW/10.f *C_SAMPLE_TIME); // Pos
	float c 	=   expf(-C_2PI* MOBS_BW      *C_SAMPLE_TIME); // Vel

	Kiso =  (a+1.f)*(b+1.f)*(c+1.f); // temporary Var
	bso =  -2.f*C_J*(a*b*c+a*b+a*c+b*c+a+b+c-7.f)       /  (     C_SAMPLE_TIME     *Kiso);
	Kso =   4.f*C_J*(3.f*a*b*c-a*b-a*c-b*c-a-b-c+3.f)   /  (powf(C_SAMPLE_TIME,2.f)*Kiso);
	Kiso = -8.f*C_J*(a-1.f)*(b-1.f)*(c-1.f)             /  (powf(C_SAMPLE_TIME,3.f)*Kiso);
	
	// Variable Initialization
	clear();
}


void MotionObserver::run()
{
	del_theta_err = norm_angle(theta_k - theta_km1) - del_theta_kp1;
	

	M_d_o				= bso/C_SAMPLE_TIME*del_theta_err;
	M_p_o				+= Kso*del_theta_err;
	M_i_o				+= M_p_o* Kiso/Kso * C_SAMPLE_TIME;

	T_d_hat				= M_d_o + M_p_o + M_i_o;
	T_hat 				= Tem_com + T_d_hat;

	omega_hat_kp1	   += (C_SAMPLE_TIME/C_J)*T_hat;
	
	del_theta_kp1 		= C_SAMPLE_TIME*0.5f*(omega_hat_kp1 + omega_hat_k);

	theta_hat_kp1       = norm_angle(theta_hat_k + del_theta_kp1);
	
	omega_hat_k 		= omega_hat_kp1;								//Set up for next sample period
	theta_hat_k 		= theta_hat_kp1;								//Set up for next sample period
	return;
}

void  	MotionObserver::update(const float theta, const float Tem)
{
	theta_km1   =   theta_k;
	theta_k		= 	theta;
	Tem_com 	=	Tem;
	run();
	return;
}
void 	MotionObserver::clear()
{
	theta_km1   		= 0.f;
	theta_k 			= 0.f;
	theta_hat_k  		= 0.f;
	theta_hat_kp1 		= 0.f;
	omega_hat_k 		= 0.f;
	omega_hat_kp1 		= 0.f;
	Tem_com 			= 0.f;
	M_i_o 				= 0.f;
	T_d_hat  			= 0.f;
	M_p_o 				= 0.f;
	M_d_o				= 0.f;
	T_hat 				= 0.f;
	del_theta_err 		= 0.f;
}
float 	MotionObserver::get_theta_kp1() const
{
	return theta_hat_kp1;
}
float 	MotionObserver::get_omega_kp1() const
{
	return  omega_hat_kp1;
}
float		MotionObserver::get_gains(const int& i) const
{
	if (i == 1) return bso;
	if (i == 2) return Kso;
	if (i == 3) return Kiso;
	return -1.f;
}
