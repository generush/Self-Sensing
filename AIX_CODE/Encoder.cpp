// Encoder.cpp
// 04/18/15
// Marc Petit
#include "Encoder.h"


Encoder::Encoder()
{
	ENC_Count 				= 0.f;
	first_zp 				= 1.f;
	encoder_count_offset_ZP = 0.f;
	encoder_count_offset_SF = 0.f;// -30.f;
}
void  Encoder::update()
{
	//  Update Position
	#ifdef VDSP
		encoder_count  = ENCOD_A.POS_A;

		if(ENCOD_A.STAT_A.ZP&&first_zp)
		{
			encoder_count_offset_ZP = last_encoder_count;
			encoder_count_offset_SF = 1339;
			first_zp = 0;
		}	
		encoder_count = encoder_count  + encoder_count_offset_SF;
		
		if(Flag_init_pos)
		{
			// ENC_Count++;
			// if (ENC_Count == 10000) // Wait one sec after alignment start
			// {
				// encoder_count_offset_SF = -encoder_count;	
			// }
			// if (ENC_Count == 50000)
			// {
				// ENC_Count = 50000;   
			// }
		}		
		
		if(encoder_count >= C_CPR)
		{
			encoder_count = encoder_count - C_CPR;
		}
		if(encoder_count < 0.f)
		{
			encoder_count = encoder_count + C_CPR;
		}
		last_encoder_count = encoder_count;
		
		if(motor_pos)
		{         	
			theta_rm_A 	= norm_angle(encoder_count*C_RAD_PER_COUNT);
			theta_r_A 	= norm_angle(C_P_A*theta_rm_A);
			theta_rm_B 	= norm_angle(theta_rm_A - C_THETA_DIFF);
			theta_r_B 	= norm_angle(C_P_B*theta_rm_B);
		}	
		else
		{      
			theta_rm_B 	= norm_angle(encoder_count*C_RAD_PER_COUNT);
			theta_r_B 	= norm_angle(C_P_B*theta_rm_B);
			theta_rm_A 	= norm_angle(theta_rm_B + C_THETA_DIFF);
			theta_r_A 	= norm_angle(C_P_A*theta_rm_A);
		}
	#else
		//#warning "Simulink Input"
		extern float theta_sim; 
		if(motor_pos)
		{         	
			theta_rm_A 	= norm_angle(theta_sim);
			theta_r_A 	= norm_angle(C_P_A*theta_rm_A);
			theta_rm_B 	= norm_angle(theta_rm_A - C_THETA_DIFF);
			theta_r_B 	= norm_angle(C_P_B*theta_rm_B);
		}	
		else
		{      
			theta_rm_B 	= norm_angle(theta_sim);
			theta_r_B 	= norm_angle(C_P_B*theta_rm_B);
			theta_rm_A 	= norm_angle(theta_rm_B + C_THETA_DIFF);
			theta_r_A 	= norm_angle(C_P_A*theta_rm_A);
		}
	#endif
}

// I dont understand the reason for ENC_Count
void  Encoder::clear()
{
		ENC_Count 				= 0.f;
}
