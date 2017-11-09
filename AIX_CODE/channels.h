#pragma             once


#ifdef _Chirp_Gen
	#define             CHANNEL_0	v_qs_r_com_B
	#define             CHANNEL_1	I_qs_r_B
	#define             CHANNEL_2   0 // theta_rm_hat_B
	#define             CHANNEL_3	0  // theta_rm_B
	#define             CHANNEL_4	0
	#define             CHANNEL_5	0
	
	
	
#else
    // SPMSM Test Config
	//#define             CHANNEL_0	I_ds_r_B
	//#define             CHANNEL_1	I_qs_r_B
	//#define             CHANNEL_2   theta_rm_A // theta_rm_hat_B
	//#define             CHANNEL_3	theta_rm_B  // theta_rm_B
	//#define             CHANNEL_4	v_ds_r_com_B
	//#define             CHANNEL_5	v_qs_r_com_B

	// LDQ Measurement
	#define             CHANNEL_0	v_ds_r_com_B
	#define             CHANNEL_1	v_qs_r_com_B
	#define             CHANNEL_2   I_ds_r_B // theta_rm_hat_B
	#define             CHANNEL_3	I_qs_r_B  // theta_rm_B
	#define             CHANNEL_4	w_r_hat_B
	#define             CHANNEL_5	theta_rm_B

    // Torque Ripple Config
	// #define             CHANNEL_0	v_ds_r_com_B
	// #define             CHANNEL_1	I_ds_r_B
	// #define             CHANNEL_2   lambda_d_hat_kp1 // theta_rm_hat_B
	// #define             CHANNEL_3	lambda_q_hat_kp1  // theta_rm_B
	// #define             CHANNEL_4	T_e_est
	// #define             CHANNEL_5	theta_i
#endif




#define CH_CODE(channel)  CH_CODE2(channel)
#define CH_CODE2(channel) s ##channel





enum chan{
	s0,
	sI_ds_r_B,
	sI_qs_r_B,
	slambda_d_hat_kp1,
	slambda_q_hat_kp1,
	sT_e_est,
	sT_e_hat_kp1,
	stheta_i,
	stheta_rm_A,
	stheta_rm_B,
	stheta_rm_hat_B,
	sv_ds_r_com_A,
	sv_qs_r_com_A,
	sv_ds_r_com_B,
	sv_qs_r_com_B,
	sw_r_hat_B,
	sw_rm_hat_kp1
};



