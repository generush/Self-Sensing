/*************************************************************************/
/*						IPM Control Program - IPMSM.h					                   */
/*						v1.0											                                 */
/*						03/27/10											                             */
/*						Wei Xu							                                       */
/*************************************************************************/
/* v1.1  First Version   Implemented based on Jae Suk's code             */
/*        03/27/10                                                       */
/* v1.2  Integrated Selfsensing into DB-DTFC                             */
/*       Transition between HFI to BEMF Tracking   (not tested)          */
/* v1.3  Tyler and Marc's implementation of extended back emf tracking   */
/*   			into DB-DTFC developed by Wei Xu and Jaesuk Lee          */
/*				10/23/2013												 */
/*                                                                       */
/*************************************************************************/
#pragma once
#include "constants.h"
#include "channels.h"
#include "hardware.h"

#ifdef VDSP
	#include"ethernet.h"
	#include"init_pwm.h"
	#include <irq.h>
	//#include "serial.h"
	#include <XCSDataLink.h>
	#include <I2C.h>
	//#include <shared/extract.h>
	//#include <shared/nop.h>
	#include <shared/lib_clip.h>
	//#include <terminal.h>
#else
	struct PWM_sim PWM_B, PWM_C;
	const int num_samples       = C_SAMPLES;
#endif



#include "AnalogInput.h"
#include "Encoder.h"

#include "constants.h"

#include "FOC.h"
#include "MotionObserver.h"
#include "MotionController.h"

#include <math.h>



float norm_angle(float angle);
static float DTC(const float& I_ph);
static void PWM(volatile float * Uout, const float& v_d_com, const float& v_q_com, const float& theta_r, const float& Inv_Vdc);

//------Motor to use for aligning rotor and initial position------//
const int motor_pos                              = IPMSM;	// 0 = Alignment using Motor B (IPM),  1 = Alignment using Motor A (SPM)
const int speed_ctrl                             = IPMSM;	// 0 = IPM speed control, 1 = SPM speed control

int algn_cnt = 0;

//------Prevent divides------//
const float Sample_time_o_J                      = C_SAMPLE_TIME/C_J;
const float Inv_Kt_A                             = 1/C_KT_A;
const float Inv_Kt_B                             = 1/C_KT_B;

/* Structures and defined Types */
const float pwm_frequency                        = 10000.f;				// switching frequency of the PWM
const float dt                                   = 1/pwm_frequency;					// switching period



/* PWM Compensation */
//------Deadtime Compensation Look-up-table------//
const float DTC_LUT[41]                          = {-0.03075000000000f,-0.03069625159420f,-0.03063875478261f,-0.03057813217391f,-0.03051500637681f,-0.03045000000000f,-0.03038000000000f,-0.03015000000000f,-0.02985000000000f,-0.02960000000000f,-0.02915000000000f,-0.02884679676441f,-0.02863248331648f,-0.02841977148635f,-0.02812137310415f,-0.02765000000000f,-0.02615000000000f,-0.02315000000000f,-0.01765000000000f,-0.00685000000000f,0.00235000000000f,0.00825000000000f,0.02060000000000f,0.02440000000000f,0.02705000000000f,0.02805000000000f,0.02841407649202f,0.02865247013997f,0.02882882554191f,0.02900678729589f,0.02925000000000f,0.02960000000000f,0.02995000000000f,0.03030000000000f,0.03035000000000f,0.03035000000000f,0.03036813333333f,0.03042040000000f,0.03050360000000f,0.03061453333333f,0.03075000000000f};	


/* Drive Flags */
enum {
    STARTUP,
    RUN,
    STOP,
    INIT,
    ALIGN
};	

volatile int Align_cnt                           = 0;

volatile int drive_flag                          =  STOP;	   			

#ifdef _FOC_A
FOC 			FOC_A(SPMSM);
#endif
#ifdef _FOC_B
FOC 			FOC_B(IPMSM);
#endif
Encoder 	   	ENC;
AnalogInput		AI;
MotionObserver MO;
MotionController MC;
	


/* Flags */
	int Flag_fault_A                             = 0;
	int Flag_fault_B                             = 0;
	int Flag_first_step                          = 1;
	int	Flag_run                                 = 0;
	int Flag_init_pos                            = 0;
	int Flag_discharge                           = 0;
	int Flag_align                               = 0;
	int Flag_rec_offset                          = 0;
	int Flag_rec                                 = 0;
	volatile long dump_cnt                       = 0;
	int Flag_source                              = 0;
	int Flag_ID                                  =0;
	int Flag_pulse                               = 1;   // 1 means current on, 0 current off
	int Flag_prev                                = 0;		// sets previous so can 'hold' current state
	
//------3 phase converter variable------//
	volatile float     Uout_A[3+1]               = {0.f, 0.f, 0.f, 0.f};      // calculated output duty 
	volatile float     Uout_B[3+1]               = {0.f, 0.f, 0.f, 0.f};      // calculated output duty 	  


	

	




/********************************************************************************************************************************/
//------Menu variables------//							  							  


//------Set PWM------//	
    static float v_as_com                        = 0.f,v_bs_com = 0.f,v_cs_com = 0.f;	

//-----Measured currents for inverter A-----//
	float I_ds_r_A                               = 0.f, I_qs_r_A = 0.f;
		

//-----Measured currents for inverter B-----//
	float I_ds_r_B                               = 0.f, I_qs_r_B = 0.f;
	bool   Flag_Offset                           = 0;
		
	
//--------Counter---------//
	// float Step_Count                          = 0.f;
float Wrm_Count                                  = 0.f;
	
	// float Step_Source                         = 0.f;

//-----Motion controller------//
	int   k_sc	                                 = 1;
	float w_rm_com                               = 0.f;
	float w_rm_star                              = 0.f;		

	float T_com_A                                = 0.f;
	float T_com_B                                = 0.f;
				
//-----Current regulator------//
	float I_ds_r_com_A                           = 0.f, I_qs_r_com_A = 0.f;
	float I_ds_r_com_B                           = 0.f, I_qs_r_com_B = 0.f;
	
	
	float v_ds_r_com_B_FF                        = 0.f, v_qs_r_com_B_FF = 0.f;
	//float v_ds_r_com_B_ND                      = 0.f, v_qs_r_com_B_ND = 0.f;
	
	float v_ds_r_com_A                           = 0.f, v_qs_r_com_A = 0.f;
	float v_ds_r_com_B                           = 0.f, v_qs_r_com_B = 0.f;
	// float M_i_d_km1_A                         = 0.f, M_i_q_km1_A = 0.f;
	// float M_i_d_km1_B                         = 0.f, M_i_q_km1_B = 0.f;
	
	float I_ds_r_err_A                           = 0.f, I_qs_r_err_A = 0.f;
	float I_ds_r_err_B                           = 0.f, I_qs_r_err_B = 0.f;

	
	float M_p_d_A                                = 0.f, M_p_q_A = 0.f;
	float M_i_d_A                                = 0.f, M_i_q_A = 0.f;
	float M_p_d_B                                = 0.f, M_p_q_B = 0.f;
	float M_i_d_B                                = 0.f, M_i_q_B = 0.f;
	
	float I_ds_s_err_A                           = 0.f, I_qs_s_err_A = 0.f;
	float I_ds_s_err_B                           = 0.f, I_qs_s_err_B = 0.f;
	float I_ds_s_com_A                           = 0.f, I_qs_s_com_A = 0.f;
	float I_ds_s_com_B                           = 0.f, I_qs_s_com_B = 0.f;
	float theta_i = 0.f;
	unsigned int k_hfi                           = 1;
	unsigned int k_rec							 = 0;

//------Stationary reference frame current observer------//
	float I_ds_s_hat_B                           = 0.f, I_qs_s_hat_B = 0.f;
	float v_ds_s_com_B                           = 0.f, v_qs_s_com_B = 0.f;
	

	float I_ds_s_hat_A                           = 0.f, I_qs_s_hat_A = 0.f;
	float v_ds_s_com_A                           = 0.f, v_qs_s_com_A = 0.f; 
	
		
	
//------Rotor reference frame current observer--------//
	ComplexVar I_dq_B;
	ComplexVar I_dq_f_B;
	ComplexVar I_dq_c_B;
	
//------ Stator flux observer (Discrete time)------// Added by Jaesuk

float T_e_hat                                    = 0.f, T_e_hat_kp1 = 0.f;
float T_e_est                                    = 0.f, T_e_est_kp1 = 0.f;	
float T_em                                       = 0.f;
			
//------Position observer------//
	float w_rm_hat                               = 0.f;
	float w_rm_hat_kp1                           = 0.f;
	float theta_rm_hat_A                         = 0.f;
	float theta_rm_hat_kp1_A                     = 0.f;
	float w_rm_left                              = 0.f;
	float w_rm_left_kp1                          = 0.f;
	float M_i_o_hat_km1                          = 0.f;
	float T_d_hat_B                              = 0.f;
	float T_d_hat_A                              = 0.f;
	float theta_rm_hat_B                         = 0.f;
	float theta_rm_hat_kp1_B                     = 0.f;
	float theta_r_hat_A                          = 0.f;
	float theta_r_hat_B                          = 0.f;	

	float theta_rm_hats_B                        = 0.f;
	float theta_r_hats_B                         = 0.f;
	float w_rm_hats_kp1                          = 0.f;
	float w_rm_lefts_kp1                         = 0.f;
	
	float T_hat                                  = 0.f;	
	
	float w_r_hat_A                              = 0.f;
	float w_r_hat_B                              = 0.f;
	
	float T_em_A                                 = 0.f;		// variable used for command feedforward to the position observer
	float omega_rm_bar                           = 0.f;	// variable used for average speed

	


//------Encoder variables------//
	float theta_rm_A                             = 0.f;
	float theta_r_A                              = 0.f;
	float theta_rm_B                             = 0.f;
	float theta_r_B                              = 0.f;	
	
//------Motion Controller------//
	float T_com                                  = 0.f;
	
	//Function variables
	float ref_frame                              = 0.f;			//1 = rotor reference frame, 0 = stationary reference frame
	float theta_ref_frame_A                      = 0.f;
	float theta_ref_frame_B                      = 0.f;

//------abc to dq transformation------//
	static float alpha                           = 0.f, beta = 0.f;
	// float alpha_A                             = 0.f, alpha_B = 0.f;
	// float beta_A                              = 0.f, beta_B = 0.f;
	

//------Maximum torque per ampere LUT------//
	static float T_index                         = 0.f;
	static float percent                         = 0.f;
	static int T_index_low                       = 0;
	static int T_index_high                      = 0;
	

//------Deadtime compensation------//
	static float T_comp                          = 0.f;
							
	int Flag_test                                = 0;


//Estimated angle
float theta_A                                    = 0.f, theta_B = 0.f; //using angle for the FOC, theta_r_A from encoder, theta_r_est_A from selfsensing
float theta_r_est_A                              = 0.f, theta_r_est_B = 0.f, theta_rm_est_A = 0.f;	


//XCSDataLink
#ifdef _DATALINK
	Command command;
	UnionData data;
	volatile unsigned cmd;											
	// Command data for data linker
	volatile float cmd_data;	
	
	int dataset                                  = 0;
	
//Data Logging (Average Value)
	int Ave_Count                                = 1;
	float Torque_Ave                             = 0;
	float Lambda_Ave                             = 0;
	float w_rm_hat_Ave                           = 0;
	float Pc_Ave                                 = 0;  //Copper Loss Calculation
	float Id_hat_Ave                             = 0;
	float Iq_hat_Ave                             = 0;
#endif
	float w_rm_set                               = 0;
	float T_set_B                                = 0;
	float lambda_set                             = C_LAMBDA_PM_B;


	//int ENC_Count                              = 0;


	float v_ds_r_com_B1                          = 0;
	float v_qs_r_com_B1                          = 0;
	

	float Lq 	                                 = C_LQ_B;
	float Ld                                     = C_LD_B;
	float Ld_obs                                 = C_LD_B;
	float Lq_obs                                 = C_LQ_B;
	float lambda_pm_obs                          = C_LAMBDA_PM_B;	
	
	float tau_d 	                             = Ld_obs/C_RS_B;
	float tau_q 	                             = Lq_obs/C_RS_B;
	float Inv_RS_B 	                             = 1.f/C_RS_B;
	
	float Ld_set                                 = 0.0f;
	float Lq_set                                 = 0.0f;
	float lambda_pm_set                          = 0.0f;
	float lambda_pm                              = C_LAMBDA_PM_B;
	
		
	
	float T_e_hat_kp11                           = 0.0f;
	float lambda_qs_r_B1                         = 0.0f;
	float lambda_ds_r_B1                         = 0.0f;
	

	
	// ------- variables for benchmark parameter estimation -----///
	float Rs_hat_bench                           = 0.f;
	float Ls_hat_bench                           = 0.f;
	
	// Chirp
	float chirp                                  = 0.f;                   // Chirp
	
