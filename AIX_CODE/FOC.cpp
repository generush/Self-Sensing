// MotionObserver.cpp
// Marc Petit
// 04/20/15  
#include <math.h>

#include "FOC.h"

/*
	Input:
*/
FOC::FOC(bool machine)
{
	if(machine           == IPMSM)
	{
		Ldq 		     = ComplexVar(C_LD_B,C_LQ_B);
		lambda_pm 	     = C_LAMBDA_PM_B;
		Inv_Kt 		     = 1.f/C_KT_B;
		// K_P  		     = ComplexVar(C_K_P_D_B, C_K_P_Q_B);
		// K_I			     = ComplexVar(C_K_I_D_B*C_SAMPLE_TIME, C_K_I_Q_B*C_SAMPLE_TIME);
		
		BW               = expf(-2.f*C_PI*FOC_BW*C_SAMPLE_TIME);
		K	             = (1.f-BW)*C_RS_B/(ComplexVar(1.f,1.f)-expc(-C_RS_B/Ldq*C_SAMPLE_TIME));
		K_P              = K.comp_mul(expc(-C_RS_B/Ldq*C_SAMPLE_TIME));
		K_I              = (K - K_P);
		
		//K_P              = K_P.comp_mul(ComplexVar(0.1f,1.f));
		//K_I              = K_I.comp_mul(ComplexVar(0.1f,1.f));
	}
	else
	{
		Ldq 		     = ComplexVar(C_LS_A,C_LS_A);
		lambda_pm 	     = C_LAMBDA_PM_A;
		Inv_Kt 		     = 1.f/C_KT_A;
		K_P  		     = ComplexVar(C_K_P_D_A, C_K_P_Q_A);
		K_I			     = ComplexVar(C_K_I_D_A*C_SAMPLE_TIME, C_K_I_Q_A*C_SAMPLE_TIME);	
	}
}
void  	FOC::update(const float& Tcom, const ComplexVar& Idq_k, const float& w_r_k)
{
	I_dq                 = Idq_k;
	T_com                = Tcom;
	w_r                  = w_r_k;
	run();
}
void  	FOC::run()
{
    //if( T_com > 3.f) 
    //{
    //	T_com         = 3.f;
    //}
    //else if(T_com < -3.f)
    //{
    //	T_com         = -3.f;
    //}	
	I_dqs_r_com          = ComplexVar(0.f,T_com*Inv_Kt);
	I_dq_err             = I_dqs_r_com - I_dq;
	M_p_dq               = I_dq_err.comp_mul(K_P);
	M_i_dq               = M_i_dq + I_dq_err.comp_mul(K_I);

	//v_dqs_r_dec          = ComplexVar(0,1)*(Ldq.comp_mul(I_dq) + ComplexVar(lambda_pm,0))*w_r;
	v_dqs_r_com          =  M_p_dq + M_i_dq + v_dqs_r_dec;					// d-axis stator voltage with decoupling 
}

void   FOC::clear()
{
		T_com            = 0.f, w_r = 0.f;
		I_dqs_r_com.clear();
		I_dq.clear();
		I_dq_err.clear();
		v_dqs_r_com.clear();
		M_p_dq.clear();
		M_i_dq.clear();
		v_dqs_r_dec.clear();
}

float   FOC::get_v_d_com()
{
	return v_dqs_r_com.get_d_comp();
}
float   FOC::get_v_q_com()
{
	return v_dqs_r_com.get_q_comp();
}
float		FOC::get_gains(const int& i) const
{
	if (i == 1) return K_P.get_d_comp();
	if (i == 2) return K_P.get_q_comp();
	if (i == 3) return K_I.get_d_comp();
	if (i == 4) return K_I.get_q_comp();
	return -1.f;
}
