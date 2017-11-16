// MotionController.cpp
// Marc Petit
// 03/28/15 
#include "MotionController.h"

/*
	Input:
*/
MotionController::MotionController()
{
	float a		=	expf(-C_2PI* MC_BW/100.f*C_SAMPLE_TIME); // IPos
	float b		=   expf(-C_2PI* MC_BW/10.f *C_SAMPLE_TIME); // Pos
	float c 	=   expf(-C_2PI* MC_BW      *C_SAMPLE_TIME); // Vel

	Kva  = -C_J/C_SAMPLE_TIME*(a*b*c-1.f);					// Kva/Ts
	Kpa  =  C_J/C_SAMPLE_TIME*(2.f*a*b*c-a*b-a*c-b*c+1.f);  // Kpa
	Kipa =  -C_J/C_SAMPLE_TIME/Kpa*(a-1)*(b-1)*(c-1);		// Kipa*Ts/Kpa

	// Kva = 0.0222f; // 20 Hz
	// Kpa = 2.7849e-5f; // 2Hz
	// Kipa = 1.1322e-4f; // 0.2Hz

	// 40 Hz
	// Kva = 0.0880f; // 40 Hz  // Kva/Ts
	// Kpa = 2.2124e-04f; // 4Hz // Kpa
	// Kipa = 2.2645e-04f; // 0.4Hz // Kipa*Ts/Kpa
	
	//30 Hz
	// Kva = 0.0663f; // 40 Hz  // Kva/Ts
	// Kpa = 1.2488e-04f; // 4Hz // Kpa
	// Kipa = 1.6983e-04f; // 0.4Hz // Kipa*Ts/Kpa
	
	//25 Hz
	// Kva = 0.0553f; // 40 Hz  // Kva/Ts
	// Kpa = 8.6876e-05f; // 4Hz // Kpa
	// Kipa = 1.4152e-04f; // 0.4Hz // Kipa*Ts/Kpa
	
	// 20 Hz
	//Kva = 0.0443f; // 40 Hz  // Kva/Ts
	//Kpa = 5.5698e-05f; // 4Hz // Kpa
	//Kipa = 1.1322e-04f; // 0.4Hz // Kipa*Ts/Kpa

	// 15 Hz
	// Kva = 0.0333f; // 40 Hz  // Kva/Ts
	// Kpa = 3.1385e-05f; // 4Hz // Kpa
	// Kipa = 8.4912e-05f; // 0.4Hz // Kipa*Ts/Kpa
	
	// 10 Hz
	// Kva = 0.0222f; // 40 Hz  // Kva/Ts
	// Kpa =  1.3974e-05f; // 4Hz // Kpa
	// Kipa = 5.6607e-05f; // 0.4Hz // Kipa*Ts/Kpa
	
	// 5 Hz
	//Kva = 0.0765f; // 40 Hz  // Kva/Ts
	//Kpa =  1.3974e-004f; // 4Hz // Kpa
	//Kipa = 2.26428e-004; // 0.4Hz // Kipa*Ts/Kpa
	//Kva =  0.0015f; // 40 Hz  // Kva/Ts
	//Kpa =  0.f; // 4Hz // Kpa
	//Kipa = 0.f; // 0.4Hz // Kipa*Ts/Kpa

	
	//#define C_B_A 						0.0265f				// 20 Hz
    //#define C_K_A 						0.6658f				// 4 Hz
    //#define C_K_IA						2.7f				// 0.8 Hz
	
	// Variable Initialization
	clear();
}


void MotionController::run()
{
	omega_err = omega_com - omega_k; 

	M_v   = Kva  * omega_err;
	M_p  += Kpa  * omega_err;
	M_ip += Kipa * M_p;

	Tem_com = M_v + M_p + M_ip;
	return;
}

void  	MotionController::clear()
{
	omega_com		= 0.f;
	omega_k			= 0.f;
	Tem_com			= 0.f;
	M_ip			= 0.f;
	M_p				= 0.f;
	M_v				= 0.f;
	omega_err		= 0.f;
}

void  	MotionController::update(const float new_omega_com, const float new_omega_k)
{
	omega_com	= 	new_omega_com;
	omega_k 	=	new_omega_k;
	run();
	return;
}
float 	MotionController::get_Tem_com() const
{
	return Tem_com;
}
float		MotionController::get_gains(const int& i) const
{
	if (i == 1) return Kva;
	if (i == 2) return Kpa;
	if (i == 3) return Kipa;
	return -1.f;
}

