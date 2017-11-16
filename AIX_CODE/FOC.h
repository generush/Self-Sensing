// FOC.h
// Marc Petit
// 04/20/15 

# include "constants.h"
# include "ComplexVar.h"



class FOC 
{
	private:
		// Member functions
		void  	run();
		float   BW;
		float   T_com, w_r,Inv_Kt;
		float   lambda_pm;
		ComplexVar Ldq;
		ComplexVar I_dq_err, v_dqs_r_dec;
		ComplexVar I_dqs_r_com, I_dq, v_dqs_r_com;
		ComplexVar M_p_dq,M_i_dq;
		ComplexVar K_I, K_P, K;
	public:
		FOC(bool machine);
		void  	update(const float& Tcom, const ComplexVar& Idq_k, const float& w_r_k);
		void 	clear();
		float   get_v_d_com();
		float   get_v_q_com();
		float		get_gains(const int& i) const;
};
