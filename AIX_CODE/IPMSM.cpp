/*************************************************************************/
/*						IPM Control Program - IPMSM.cpp					 */
/*						v1.0			                                 */
/*						03/25/15										 */
/*						Marc Petit			                             */ 
/*************************************************************************/
//Pre-Compiler Macros
#define _FOC_A
#define _FOC_B
// #define _DTC_A					// dead time compensation for SPM
// #define _DTC_B					// dead time comp for IPM
#define _MotionObserver


/*************************************************************************/

#include "IPMSM.h"


#ifdef VDSP
	extern void menu_init ();
	extern void menu_serve();
	const unsigned motion_control_serve = 80;
    const float one_by_motion_control_serve = 1 / (float)motion_control_serve;
    volatile unsigned motion_control_count = 0;	
#endif

int counter = 0;
/**************************************************************************/
/*                                                                        */
/*      XCB2000 - Backplane is equipped with the following Cards:         */
/*                                                                        */
/**************************************************************************/
/*                                                                        */
/*  Slot1: XCS2000 - control card                                         */
/*  Slot2: XCC2000 - communication card for TCP/IP interface              */
/*  Slot3: empty                                                          */
/*  Slot4: XCI2000 - interface card for 3 phase converter A               */
/*  Slot5: XCI2000 - interface card for 3 phase converter B               */
/*  Slot6: empty                                                          */
/*  Slot7: XCI2020 - universal I/O and oscilloscope output                */
/*                                                                        */
/**************************************************************************/
					

/**************************************************************************/
/**************************************************************************/
/** name:     control_ISR      (Current Controller)                      **/
/**                                                                      **/
/** function: Interrupthandler for A-to-D conversion ready               **/
/**           here the user implements his own processing routine        **/
/**************************************************************************/
/**************************************************************************/
#ifdef VDSP
void control_ISR (irq_param)
#else
void control_ISR ()
#endif
{
	// Sample Analog Inputs?
	#ifdef VDSP
		da_convert(AN0,AN07);
		da_trigger(AN0_15_);	
	#endif
	//  Update Position
	ENC.update();
	theta_rm_A 	= ENC.theta_rm_A;
	theta_r_A 	= ENC.theta_r_A;
	theta_rm_B 	= ENC.theta_rm_B;
	theta_r_B 	= ENC.theta_r_B;

	#ifdef VDSP
		AN0_15.ADready.ack();   
		AN16_31.ADready.ack();  
		ad_convert(AN8,AN31);
	#endif
	AI.update();
	//Protection routine
	if((fabs(w_rm_left_kp1) > C_WM)  && drive_flag == RUN)
	{
		Flag_fault_A |= C_FT_WM;
		Flag_fault_B |= C_FT_WM;
	}

	if(fabs(AI.Ias_A) > C_OC_A) Flag_fault_A |= C_FT_OC_IAS;
	if(fabs(AI.Ibs_A) > C_OC_A) Flag_fault_A |= C_FT_OC_IBS;
	if(fabs(AI.Ics_A) > C_OC_A) Flag_fault_A |= C_FT_OC_ICS;
	  
	if(AI.Vdc_A_fil > C_OV) Flag_fault_A |= C_FT_OV;
	if(AI.Vdc_A_fil < C_UV) Flag_fault_A |= C_FT_UV;

	if(fabs(AI.Ias_B) > C_OC_B) Flag_fault_B |= C_FT_OC_IAS;
	if(fabs(AI.Ibs_B) > C_OC_B) Flag_fault_B |= C_FT_OC_IBS;
	if(fabs(AI.Ics_B) > C_OC_B) Flag_fault_B |= C_FT_OC_ICS;
	  
	if(AI.Vdc_B_fil > C_OV) Flag_fault_B |= C_FT_OV;
	if(AI.Vdc_B_fil < C_UV) Flag_fault_B |= C_FT_UV;

	if(Flag_run) 
	{
		PWM_B.CH7 = 1;		//Turn on Initial charge switch
		PWM_C.CH7 = 1;		// = Directly connected to diode
	}	
	if(AI.Vdc_B_fil > C_BRAKE_ON) 	
	{
		PWM_B.CH6 = 0;		//Turn on brake
		PWM_B.CH7 = 0;		//Turn off initial charge switch
	}    	
	else
	{
		PWM_B.CH6 = 1;		//Turn off brake
	}
	if(AI.Vdc_A_fil > C_BRAKE_ON) 	
	{
		PWM_C.CH6 = 0;		//Turn on brake
		PWM_C.CH7 = 0;		//Turn off initial charge switch
	}  	
	else 
	{
		PWM_C.CH6 = 1;		//Turn off break
	}

	//------Fault Check routine------//	---> stop motor
	if((!Flag_run)|Flag_fault_A|Flag_fault_B)
	{
		PWM_B.CH0 = 1;			//Turn off all switches for inverter A
		PWM_B.CH1 = 0;
		PWM_B.CH2 = 1;
		PWM_B.CH3 = 0;
		PWM_B.CH4 = 1;
		PWM_B.CH5 = 0;
		PWM_B.CH6 = 1;
		PWM_B.CH7 = 0;
		
		PWM_C.CH0 = 1;			//Turn off all switches for inverter B
		PWM_C.CH1 = 0;
		PWM_C.CH2 = 1;
		PWM_C.CH3 = 0;
		PWM_C.CH4 = 1;
		PWM_C.CH5 = 0;
		PWM_C.CH6 = 1;
		PWM_C.CH7 = 0;			
		
		Flag_run = 0;			
	}     
	#ifdef VDSP
		if(Flag_run && !Flag_align)
		{         
				
				display_gotoxy(0,3);
				display_putstring("drive_flag = RUN");  
		}   
		else
		{  
			//------If already in STOP------//	  	    
			if(drive_flag == RUN)					//Run
			{	
				display_gotoxy(0,3);
				display_putstring("drive_flag = RUN"); 
				Flag_run = 1;
				Flag_align = 0;
				Flag_ID = 1;   
				Flag_init_pos = 0;
				Flag_source = 1;
			}   
			if(drive_flag == STARTUP)		//startup, initial Rotor position estimation
			{
				display_gotoxy(0,3);
				display_putstring("drive_flag = STARTUP");
				Flag_run = 1;
				Flag_align = 1;
				Flag_ID=0;
				Flag_init_pos = 1;
				Flag_source = 0;
			}   
		}
		if(drive_flag == STOP)				//STOP and clear faults
		{
			display_gotoxy(0,3);
			display_putstring("drive_flag = STOP");
			Flag_fault_A = 0;
			Flag_fault_B = 0;
			Flag_run = 0;
			Flag_align = 0;
			Flag_init_pos = 0;
			Flag_ID=0;
			
			//Clear the Encoder counter
			#ifdef _FOC_A
				FOC_A.clear();
			#endif
			#ifdef _FOC_B
				FOC_B.clear();
			#endif

			ENC.clear();
			MO.clear();
			MC.clear();
				
			//Clear voltage commands
			v_ds_r_com_A = 0.f;
			v_qs_r_com_A = 0.f;
			v_ds_r_com_B = 0.f;
			v_qs_r_com_B = 0.f;
			v_ds_r_com_B1 = 0.f;
			v_qs_r_com_B1 = 0.f;
			v_ds_r_com_B_FF = 0.f;
			v_qs_r_com_B_FF = 0.f;

					
			//Clear current commands
			I_ds_r_com_A = 0.f;
			I_qs_r_com_A = 0.f;
			I_ds_r_com_B = 0.f;
			I_qs_r_com_B = 0.f;
					
			//I_ds_r_hat_A = 0.f;
			//I_qs_r_hat_A = 0.f;
			
			I_dq_B.clear();
			I_dq_f_B.clear();
			I_dq_c_B.clear();
			
			T_e_hat_kp1 = 0.f;
			T_e_est_kp1 = 0.f;

			T_com_A = 0.f;
			T_com_B = 0.f;
			

			w_rm_left_kp1 = 0.f;
			w_rm_hat_kp1 = 0.f;
			theta_rm_hat_kp1_A =0.f;
			theta_rm_hat_B = 0.f;

		}    
	#endif/*******************************************************************************************************************/  	   
	//------Converting currents from dq stator to rotor reference frame(Inverter A)------//	
	I_ds_r_A = AI.I_ds_s_A*cosf(theta_r_A) + AI.I_qs_s_A*sinf(theta_r_A);
	I_qs_r_A = AI.I_qs_s_A*cosf(theta_r_A) - AI.I_ds_s_A*sinf(theta_r_A); 

	//------Converting currents from dq stator to rotor reference frame(Inverter B)------//	
	I_ds_r_B = AI.I_ds_s_B*cosf(theta_r_hat_B) + AI.I_qs_s_B*sinf(theta_r_hat_B);
	I_qs_r_B = AI.I_qs_s_B*cosf(theta_r_hat_B) - AI.I_ds_s_B*sinf(theta_r_hat_B); 
	
	
	
	I_dq_B   = ComplexVar(I_ds_r_B,I_qs_r_B);

	T_e_est = 3*(C_LAMBDA_PM_B*I_qs_r_B + (C_LD_B - C_LQ_B)*I_ds_r_B*I_qs_r_B);	/********************************************************************************************************/   
	//Alignment
	if(Flag_run&&Flag_align)
	{	
		ref_frame = 0.f;		//Stator reference frame
		algn_cnt++;
		if(ENC.first_zp)
		{
			if(motor_pos)
			{
				v_ds_r_com_B = 0.f;
				v_qs_r_com_B = 0.f;
				v_ds_r_com_A = 3*sinf(2*C_2PI*algn_cnt*C_SAMPLE_TIME);
				v_qs_r_com_A = 3*cosf(2*C_2PI*algn_cnt*C_SAMPLE_TIME);
			}
			else
			{
				v_ds_r_com_A = 0.f;
				v_qs_r_com_A = 0.f;
				v_ds_r_com_B = 3*sinf(2*C_2PI*algn_cnt*C_SAMPLE_TIME);
				v_qs_r_com_B = 3*cosf(2*C_2PI*algn_cnt*C_SAMPLE_TIME);
			}
		}
		else
		{
			if(motor_pos)
			{
				I_ds_s_com_A = 3.f;
				I_qs_s_com_A = 0.f;
				I_ds_s_com_B = 0.f;
				I_qs_s_com_B = 0.f;
			}
			else
			{
				I_ds_s_com_A = 0.f;
				I_qs_s_com_A = 0.f;
				I_ds_s_com_B = 3.f;
				I_qs_s_com_B = 0.f;
			}

			//------Motor A Current regulator------//
			I_ds_s_err_A = I_ds_s_com_A - AI.I_ds_s_A;
			I_qs_s_err_A = I_qs_s_com_A - AI.I_qs_s_A;

			M_p_d_A = C_K_P_D_A*I_ds_s_err_A;
			M_p_q_A = C_K_P_Q_A*I_qs_s_err_A;

			M_i_d_A += (C_K_I_D_A*I_ds_s_err_A)*C_SAMPLE_TIME;
			M_i_q_A += (C_K_I_Q_A*I_qs_s_err_A)*C_SAMPLE_TIME;

			v_ds_r_com_A = M_p_d_A + M_i_d_A;
			v_qs_r_com_A = M_p_q_A + M_i_q_A;	
					
			// ------Motor B------//
			I_ds_s_err_B = I_ds_s_com_B - AI.I_ds_s_B;
			I_qs_s_err_B = I_qs_s_com_B - AI.I_qs_s_B;

			M_p_d_B = C_K_P_D_B*I_ds_s_err_B;
			M_p_q_B = C_K_P_Q_B*I_qs_s_err_B;

			M_i_d_B += (C_K_I_D_B*I_ds_s_err_B)*C_SAMPLE_TIME;
			M_i_q_B += (C_K_I_Q_B*I_qs_s_err_B)*C_SAMPLE_TIME;
									
			v_ds_r_com_B = M_p_d_B + M_i_d_B;
			v_qs_r_com_B = M_p_q_B + M_i_q_B;	
		}
	}


	//*****************************************************************************************************//	
	//Speed Trajectory
	if(Flag_run&&!Flag_align)
	{
		if(Wrm_Count >= 6000)
		{
			if (w_rm_set >= w_rm_com)
			{
				w_rm_com = w_rm_com + 0.05f;	 
				if(w_rm_com >= w_rm_set) 
		
				{
					w_rm_com = w_rm_set;
				}
			}
			else
			{
				w_rm_com = w_rm_com - 0.05f;	 	
				if(w_rm_com <= w_rm_set) 
				{
					w_rm_com = w_rm_set;
				}
			}
		}
		else
		{
		   Wrm_Count++; 
		}
	}
	else
	{
		Wrm_Count = 0.f;
		w_rm_com = 0.f;    
	}

	/**************************************************************************************************/
	//Torque Trajectory
	if(Flag_run&&!Flag_align)
	{	
		// do this if torque command is going to motor A (SPM)
		if (!speed_ctrl)  // IPMSM in speed_ctrl
		{
			if (T_set_B >= T_com_A)
					{
						T_com_A= T_com_A + 0.00002f;	
						if(T_com_A >= T_set_B) 
						{
							T_com_A = T_set_B;
						}
					}
					else
					{
						T_com_A= T_com_A - 0.00002f;	 
				
						
						if(T_com_A <= T_set_B) 
			
						{
							T_com_A = T_set_B;
						}
					}
		  }
		  else if (speed_ctrl)  // SPMSM in speed_ctrl
		  {
					if (T_set_B >= T_com_B)
					{
						T_com_B= T_com_B + 0.0002f;	 
						if(T_com_B >= T_set_B) 
						{
							T_com_B = T_set_B;
						}
					}
					else
					{
						T_com_B= T_com_B - 0.0002f;	 
						if(T_com_B <= T_set_B) 
						{
							T_com_B = T_set_B;
						}
					}
		  } 
	}
		
	/*********************************************************************************************************/	


	/***********************************************************************************/


	//------ Motion Observer------//
	#ifdef _MotionObserver
		if(speed_ctrl)
		{
			// T_e_est = C_3_O_2*C_P_A*I_qs_r_A*C_LAMBDA_PM_A;
			MO.update(theta_rm_A, T_com_A);
			theta_rm_hat_A = MO.get_theta_kp1();
			w_rm_hat_kp1   = MO.get_omega_kp1();
			w_rm_left_kp1  = MO.get_omega_left_kp1();
			theta_r_hat_A  = norm_angle(theta_rm_hat_A * C_P_A);
			theta_rm_hat_B = norm_angle(theta_rm_hat_A - C_THETA_DIFF);	   				  	
			theta_r_hat_B  = norm_angle(C_P_B*theta_rm_hat_B);
			w_r_hat_A	= C_P_A*w_rm_hat_kp1;
			w_r_hat_B 	= C_P_B*w_rm_hat_kp1; //w_rm_hat_kp1 = w_rm_hat_kp1;
		}	
		else
		{	
			// T_e_est = C_Polepair_B*C_3_O_2*(lambda_pm*I_qs_r_B - (Lq - Ld)*I_ds_r_B*I_qs_r_B);
			MO.update(theta_rm_B, T_com_B);
			theta_rm_hat_B = MO.get_theta_kp1();
			w_rm_hat_kp1   = MO.get_omega_kp1();
			w_rm_left_kp1  = MO.get_omega_left_kp1();
			theta_r_hat_B  = norm_angle(theta_rm_hat_B * C_P_B);
			theta_rm_hat_A = norm_angle(theta_rm_hat_B + C_THETA_DIFF);
			theta_r_hat_A  = norm_angle(C_P_A*theta_rm_hat_A);
			w_r_hat_A	= C_P_A*w_rm_hat_kp1;
			w_r_hat_B 	= C_P_B*w_rm_hat_kp1; //w_rm_hat_kp1 = w_rm_left_kp1;
		}
	#endif
	/*********************************************************************/


	
	//------Motion Controller------//
	if(k_sc==1)
	{
		if(Flag_run&&!Flag_align)
		{
			MC.update(w_rm_com,w_rm_hat_kp1);//   w_rm_left_kp1
			T_com = MC.get_Tem_com();
			if(speed_ctrl) // SPM is in Speed Ctrl
			{
				T_com_A = T_com;  	// torque command from motion controller
				//T_com_B = T_set_B;  // torque load command from AIX
				if (Flag_rec)
				{
					// use only with chirp generator - for dynamic stiffness
					T_com_B = T_set_B; 	// need this so the IPM (B)  isn't counteracting the torque from Motor A  
				}
				I_ds_r_com_A = 0.f;
				I_qs_r_com_A = T_com_A*Inv_Kt_A;
			}
			else   // IPM is in Speed Ctrl
			{	    		
				T_com_B = T_com;
				// T_com_A = T_set_B  + chirp; False?
				I_ds_r_com_B = 0.f;
				I_qs_r_com_B = T_com_B*Inv_Kt_B;
			}			    		    	
		}	
			k_sc = 1;
	}	
	else
	{
		k_sc++;
	}	

		
	//------Current regulators------//	
	if(Flag_run&&!Flag_align)
	{				
		ref_frame = 1.f;		//Rotor reference frame
		
		#ifdef _FOC_A
			FOC_A.update(T_com_A, ComplexVar(I_ds_r_A,I_qs_r_A), w_r_hat_A);
			v_ds_r_com_A = FOC_A.get_v_d_com();
			v_qs_r_com_A = FOC_A.get_v_q_com();
		#endif	
		#ifdef _FOC_B
			FOC_B.update(T_com_B, I_dq_B, w_r_hat_B); 
			v_ds_r_com_B = FOC_B.get_v_d_com() ;
			v_qs_r_com_B = FOC_B.get_v_q_com();
		#endif

	}
	
	
	/**********************************************************************************************************/


	if(ref_frame)							//Rotor reference frame
	{
		theta_ref_frame_B = theta_r_hat_B;
		theta_ref_frame_A = theta_r_A + w_r_hat_A*C_SAMPLE_TIME*1.0f;	//theta_r_hat_A;//	//Using the 
	}
	else									//Stator reference frame
	{
		theta_ref_frame_A = 0.f;
		theta_ref_frame_B = 0.f;
	}

	/**********************************************************************************************************/
	/*************************************************************************************************************************/
	//------PWM with triplen harmonic (= Space Vector PWM) (Inverter A)------//

	//------Convert dq command voltage to abc------//	
	PWM(Uout_A,v_ds_r_com_A,v_qs_r_com_A,theta_ref_frame_A,AI.INV_Vdc_A);
		
	#ifdef _DTC_A		
		//------Deadtime compensation (Inverter A)------//		
		Uout_A[1] += DTC(AI.Ias_A);
		Uout_A[2] += DTC(AI.Ibs_A);
		Uout_A[3] += DTC(AI.Ics_A);
	#endif
	//------Clip output PWM amplitude to min/max value (Inverter A)------//	
	Uout_A[1] = lib_clip(Uout_A[1], C_MIN_AMP, C_MAX_AMP);	// 0 to 1
	Uout_A[2] = lib_clip(Uout_A[2], C_MIN_AMP, C_MAX_AMP);
	Uout_A[3] = lib_clip(Uout_A[3], C_MIN_AMP, C_MAX_AMP);
	
	PWM(Uout_B,v_ds_r_com_B,v_qs_r_com_B,theta_ref_frame_B,AI.INV_Vdc_B);
	#ifdef _DTC_B
		Uout_B[1] += DTC(AI.Ias_B);
		Uout_B[2] += DTC(AI.Ibs_B);
		Uout_B[3] += DTC(AI.Ics_B);
	#endif

	//------Clip output PWM amplitude to min/max value (Inverter B)------//	
	Uout_B[1] = lib_clip(Uout_B[1], C_MIN_AMP, C_MAX_AMP);		// 0 to 1
	Uout_B[2] = lib_clip(Uout_B[2], C_MIN_AMP, C_MAX_AMP);
	Uout_B[3] = lib_clip(Uout_B[3], C_MIN_AMP, C_MAX_AMP);

	//------Write duty cycles to A & B inverters------//
	#ifdef VDSP
		if ((!Flag_fault_A)&& (Flag_run)&&(!Flag_fault_B))
		{	
			pwm_set_combined_dutycycle(PWM_C.PAIR0, Uout_A[1]);
			pwm_set_combined_dutycycle(PWM_C.PAIR1, Uout_A[2]);
			pwm_set_combined_dutycycle(PWM_C.PAIR2, Uout_A[3]);
			
			pwm_set_combined_dutycycle(PWM_B.PAIR0, Uout_B[1]);
			pwm_set_combined_dutycycle(PWM_B.PAIR1, Uout_B[2]);
			pwm_set_combined_dutycycle(PWM_B.PAIR2, Uout_B[3]);
		}  
    #endif
//*****************************************************************************************//
	#ifdef VDSP
		//collect data for 1s
		if(Flag_first_step)
		{
			// Channel Measurements
			Value_dump[0][0]		     = CH_CODE(CHANNEL_0); 				
			Value_dump[0][1]		     = CH_CODE(CHANNEL_1);
			Value_dump[0][2]		     = CH_CODE(CHANNEL_2); 				
			Value_dump[0][3]		     = CH_CODE(CHANNEL_3); 
			Value_dump[0][4]		     = CH_CODE(CHANNEL_4); 				
			Value_dump[0][5]		     = CH_CODE(CHANNEL_5);  				
			// Controller Law
			#ifdef _DBDTFC_B 
				Value_dump[0][6]		     = 1;		//PM flux motor B
			#elif defined _FOC_B
				Value_dump[0][6]		     = 2;		//PM flux motor B
			#endif 
			// Control Bandwidth
			Value_dump[0][7]		     = FOC_BW;
			Value_dump[0][8]		     = MC_BW;
			Value_dump[0][9]		     = MOBS_BW;
			Value_dump[0][10]	         = IOBS_BW;
			Value_dump[0][11]	         = FOBS_BW;
			Value_dump[0][12]            = w_c/C_2PI;
			Value_dump[0][13]            = C_LAMBDA_C;
			Value_dump[0][14]		     = BPF_BW;
			Value_dump[0][15]		     = BPF_CF;

			// Gains
			#if defined(_FOC_B) 
			Value_dump[0][16]		     = FOC_B.get_gains(1);	
			Value_dump[0][17]		     = FOC_B.get_gains(2);
			Value_dump[0][18]		     = FOC_B.get_gains(3);
			Value_dump[0][19]		     = FOC_B.get_gains(4);
			#endif		
			Value_dump[0][20]		     = MC.get_gains(1);		
			Value_dump[0][21]		     = MC.get_gains(2);		
			Value_dump[0][22]		     = MC.get_gains(3);
			Value_dump[0][23]		     = MO.get_gains(1);		
			Value_dump[0][24]		     = MO.get_gains(2);		
			Value_dump[0][25]		     = MO.get_gains(3);
			#if defined(_FUN_CurrentObserver)  				
			Value_dump[0][26]		     = CO.get_gains(1);		
			Value_dump[0][27]		     = CO.get_gains(2);		
			Value_dump[0][28]		     = CO.get_gains(3);	
			Value_dump[0][29]		     = CO.get_gains(4);	
			#endif
			#if defined(_FUN_CurrentObserver)
			Value_dump[0][30]		     = FFO.get_gains(1);		
			Value_dump[0][31]		     = FFO.get_gains(2);
			#endif
			#if !defined(_FUN_CurrentObserver) && !defined(_I_Obs_1) 		
			Value_dump[0][32]		     = BPF.a0;	
			Value_dump[0][33]		     = BPF.a1;
			Value_dump[0][34]		     = BPF.a2;
			Value_dump[0][35]		     = BPF.b1;		
			Value_dump[0][36]		     = BPF.b2;
			#endif

			// Chirp
			#ifdef _Chirp_Gen
			Value_dump[0][37]		     = 1;
			Value_dump[0][38]		     = f_0;
			Value_dump[0][39]		     = f_1;
			Value_dump[0][40]		     = chirp_time;
			Value_dump[0][41]		     = chirp_amp;
			#endif
			#ifdef C_TLINE_INJ		
			Value_dump[0][42]		     = C_TLINE_INJ;
			#endif	
			#ifdef _HFI_B
			Value_dump[0][43]		     = 1;
			#endif		
			
			// Data Size
			Value_dump[0][44]		     = (float)   sizeof(float);	
			Value_dump[0][45]		     = (float) sizeof(Value_dump);	
			// Machine State
			Value_dump[0][46]		     = AI.Vdc_A_fil;
			Value_dump[0][47]		     = drive_flag;			
			Value_dump[0][48]		     = Flag_run;
			Value_dump[0][49]		     = Flag_align;

		}		   
		//always lof fault flags			  			   
		Value_dump[0][50]		 = Flag_fault_A;
		Value_dump[0][51]		 = Flag_fault_B;
		Value_dump[0][49]		 = ENC.encoder_count_offset_SF;
		Value_dump[0][48]		 = ENC.encoder_count_offset_ZP;
		Value_dump[0][47]		 = ENC.encoder_count_offset_SF - ENC.encoder_count_offset_ZP;
		Value_dump[0][46]		 = ENC.encoder_count_offset_SF + ENC.encoder_count_offset_ZP;
		
		if(Flag_rec == 1)
		{
		    if(k_rec++ == 1)
		    {	

				
				Value_dump[0][dump_cnt]   = CHANNEL_0;
				Value_dump[1][dump_cnt]   = CHANNEL_1;
				#ifndef _Chirp_Gen
				Value_dump[2][dump_cnt]   = CHANNEL_2;
				Value_dump[3][dump_cnt]   = CHANNEL_3;
				Value_dump[4][dump_cnt]   = CHANNEL_4;
				Value_dump[5][dump_cnt]   = CHANNEL_5;
				#endif
				dump_cnt++; k_rec = 1;
			}	
			if(	dump_cnt == num_samples)
			{
				Flag_rec = 0; 
				dump_cnt = 0;
			} 	
		}

		AN[4] = 	I_ds_r_B;
		AN[5] = 	I_qs_r_B;
		AN[6] = 	theta_rm_B/C_2PI;
		AN[7] = 	counter;//Flag_rec;
		if(counter == 1) counter = 0;
		else if(counter == 0) counter = 1;
		da_convert(AN0,AN07);
		da_trigger(AN0_15_);
		
		#ifdef _DATALINK 		
			link.Update();
		#endif
	#endif
}

// Wait for new command from Matlab
#ifdef _DATALINK 
	inline static void Wait()
	{
		delay(0.1f);
		int status = link.Read( &command, sizeof( command ) );
		cmd = command.ID;
		data.uint = command.Data;
		cmd_data = data.f;
		if( status != sizeof( command ) )
		 {
			command.ID=0;
			return;
		}
	}
#endif
/**************************************************************************/
/**************************************************************************/
/** name:     init                                                       **/
/**                                                                      **/
/** function: Init Routine for setting up all components of the system   **/
/**           here the user once declares his own configuration          **/
/**************************************************************************/
/**************************************************************************/
#ifdef _DATALINK
void init(void)
{ 
	#ifdef _DATALINK 	
		link.Open();
	//#else 
	//	AixScope_init();
	#endif
        
    //Disable all LED on the XCP2000        
    LED_off(0xFF);

    //Init serial port
    serial_init(10);

    //Disable Write protection for all secured configuration registers
    write_protect_off();

    //Disable all PWM channels
    pwm_stop(PWM_B);
    pwm_stop(PWM_C);        
        
    //Declare PWM Block configuration generator:
    //6 chanels, base frequency e.g.
    init_pwm (PWM_B, C_PWM_FREQ, C_PWM_DEADTIME);
        
    set_port (PORT_B,0xFF); // out value
    enable_output_port(PORT_B,0xFF); // dir to output

    //Declare PWM Block configuration generator:
    //6 chanels, base frequency e.g.
    init_pwm (PWM_C, C_PWM_FREQ, C_PWM_DEADTIME);
        
    set_port (PORT_C,0x00); // out value
    enable_output_port(PORT_C,0xFF); // dir to output

    enable_output_port(PORT_A,0x00); // dir to input

    set_port (PORT_D,0x00); // out value
    enable_output_port(PORT_D,0xFF); // dir to output
    
    //Internal synchronization configered: PWM_B  -> LSYNC0
    //PWM is configured as symmertric PWM -> AD-conversion will be triggered every up & down count
    pwm_config_lso(PWM_B, LSYNC0, false, true);
	//pwm_config_lso(PWM_B, LSYNC0, true, false);
	
    //Internal synchronization configered: LSYNC0 -> AD-generator
    adda_config_lsiad(AN0_15,  true, true, LSYNC0);
    adda_config_lsiad(AN16_31, true, true, LSYNC0);

    //Set up Synchronization to IRQ
    AN0_15.ADready.enable();

    //Setup Interrupthandler of AD-Converter
    AN0_15.ADready >> IRQ0;
    IRQ0 >> control_ISR;

	//Service AD-Ready interrupt flag before enabling the system
    AN0_15.ADready.ack();   

    //Now enable System    
    msc_clear_shutdown();
        
    //Wait for Capacitors to be loaded via series Resistors
    delay (0.5f);

    ENCOD_A.CONF_A.SM = true;
    ENCOD_A.CONF_A.OM = true;
    ENCOD_A.LSICFG.SELECT = 0; 
    ENCOD_A.LSICFG.RISING = true;
    ENCOD_A.LSICFG.FALLING = true;
	
	//Start PWM B
    pwm_start(PWM_B);

    //Start PWM C
    pwm_start(PWM_C);
	
    write_protect_on();

    // Start Display
    menu_init ();
    
	#ifdef VDSP
		i2c.init();
	#endif
}
#endif
#ifdef VDSP
	char hex(const char c)
	{
    return ((c>9)?'A'-10:'0') + c;
	}
#endif

/**************************************************************************/
/**************************************************************************/
/** name:     main_loop                                                  **/
/**                                                                      **/
/** function: this loop runs at the maximum speed that is left after all **/
/**           interrupt processes are served. speed may vary depending on**/
/**           the amount of computational power required in interrupts.  **/
/**************************************************************************/
/**************************************************************************/
#ifdef _DATALINK
void main_loop(void)
{
	menu_serve();
    Wait();
	switch( cmd ) {
	    case CMD_NOP:
	    {	    
			LED_on(0x01);
			break;
	    }
	   	case CMD_DISC:
	    {	    
			Flag_first_step = 1;
			drive_flag = STOP;
			break;
	    }
		case CMD_STOP:
	    {
	        drive_flag = STOP;
	        break;
	    }
	    case CMD_STARTUP:
	    {
	        drive_flag = STARTUP;
	        break;
		}
		case CMD_START:
	    {
	        drive_flag = RUN;
	        break;
		}
		case CMD_REC:
	    {
			command.ID   = CMD_REC;
			Flag_rec = (int)cmd_data;
			break;
	    }
	    case CMD_DATASET:
	    {
			command.ID   = CMD_DATASET;
			dataset=cmd_data;
			break;

	    }
	    case CMD_SEND_DATA:
	    {
	        if(Flag_first_step)
	        {
				link.Write( (const void*)&Value_dump, 52);
				Flag_first_step = 0;
	        }	        
	        else
	        {
				link.Write( (const void*)&Value_dump, num_channels*num_samples);
	        }	
			break;
	    }
	   	case CMD_SEND_DATA_SIZE:
	    {
	        command.ID   = CMD_SEND_DATA_SIZE;
			command.Data = num_samples;
			link.Write( &command, sizeof( command ) );
			break;
	    }	    
	    case CMD_SEND_NUM_CHAN:
	    {
			command.ID   = CMD_SEND_NUM_CHAN;
			command.Data = num_channels;
			link.Write( &command, sizeof( command ) );
			break;
	    }
		case CMD_TORQUE:
	    {
	        command.ID   = CMD_TORQUE;
	        data.uint = command.Data;
			T_set_B = cmd_data;
			//chirp_amp =cmd_data;
	        break;
		}
		case CMD_SPEED:
	    {
	        command.ID   = CMD_SPEED;
	        data.uint = command.Data;
			//w_rm_set = 2*C_PI*cmd_data;
			w_rm_set = cmd_data;
	        break;
		}
		case CMD_LAMBDA:
	    {
	        command.ID   = CMD_LAMBDA;
	        data.uint = command.Data;
			//lambda_set = cmd_data;
			//ENC.encoder_count_offset_SF = cmd_data;
	        break;
		}
		case CMD_WC:
	    {
	        command.ID   = CMD_WC;
	        data.uint = command.Data;
			w_c = C_2PI * cmd_data;
			#ifdef _DBDTFC_B
				DBDTFC_B.set_wc(w_c);
			#endif 
	        break;
		}
		#ifdef _Chirp_Gen
		case CMD_MINCHRIP:
	    {
	        command.ID   = CMD_LAMBDA;
	        data.uint = command.Data;
			//f_0 = cmd_data;
			//Df_by_T_c       = (f_1 - f_0)/2.f/T_c;       // Frequeny difference
	        break;
		}
		case CMD_MAXCHRIP:
	    {
		
	        command.ID   = CMD_LAMBDA;
	        data.uint = command.Data;
			//f_1 = cmd_data;
			//Df_by_T_c       = (f_1 - f_0)/2.f/T_c;       // Frequeny difference
	        break;
		}		
		#endif
		default :
		{
			break;
		}	
	}
}
#endif
float norm_angle(float angle)
{
	if(angle >= C_PI)
	{
		angle = angle - C_2PI;
	}
	if(angle < -C_PI)
	{
		angle = angle + C_2PI;
	}
	return angle;
}

inline float DTC(const float& I_ph)
{
	if(I_ph > 2.f)
	{
		T_comp = DTC_LUT[40];
	}
	else if(I_ph < -2.f)
	{
		T_comp = DTC_LUT[0];
	}
	else
	{
		T_index      = 	I_ph*10.f + 20.f;
		T_index_low  =  (int) floorf(T_index);
		T_index_high =  (int) ceilf(T_index);

		percent = (T_index - (float) T_index_low);
		T_comp = (DTC_LUT[T_index_high] - DTC_LUT[T_index_low])*percent + DTC_LUT[T_index_low];
	}
	return T_comp;
}

inline void PWM(volatile float* Uout, const float& v_d_com, const float& v_q_com, const float& theta_r, const float& Inv_Vdc)
{
	alpha = v_d_com*cosf(theta_r) - v_q_com*sinf(theta_r);
	beta  = v_d_com*sinf(theta_r) + v_q_com*cosf(theta_r);

	v_as_com = alpha;
	v_bs_com = -alpha*C_COS_PI_O_3 + beta*C_SIN_PI_O_3;
	v_cs_com = -v_as_com-v_bs_com;

	Uout[1] = v_as_com*Inv_Vdc + 0.5f;
	Uout[2] = v_bs_com*Inv_Vdc + 0.5f;
	Uout[3] = v_cs_com*Inv_Vdc + 0.5f;
}
