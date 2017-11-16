#pragma             once
// #define             VDSP
// #define             _DATALINK
#define             IPMSM 						0 
#define             SPMSM 						1

//                  how many samples to record
#define             C_SAMPLES					250000			

/*                  Analog Input macro constants */
//                  Channels based on ribbon cable and XCI2007 pin-out
#define             IA_A						AN16      //Inverter A phase A current
#define             IB_A						AN20      //Inverter A phase B current

#define             IA_B						AN8       //Inverter B phase A current
#define             IB_B						AN12      //Inverter B phase B current

#define             VDC_A						AN17      //Inverter A DC Voltage
#define             VDC_B   					AN9       //Inverter B DC Voltage


// Inverter  variables
#define             C_PWM_FREQ					10000.f		   	//Switching frequency
#define             C_PWM_DEADTIME				2.0e-6f	   		//Deadtime
#define             C_MAX_AMP					1.f  			//Maximum duty
#define             C_MIN_AMP					0.f				//Minimum duty


// Low pass filter (100 Hz BW) for DC link voltages
#define             C_V_DC_C1					0.060898632575707f			
#define             C_V_DC_C2					0.939101367424293f

// HFI Carrier 
#define             C_W_C						C_2PI*1000.f
#define             C_SPCP						C_2PI/C_W_C/C_SAMPLE_TIME // Samples per Carrier Period
#define             C_LAMBDA_C					3.0E-3f
#define 			C_TLINE_INJ					1
// Controller and Observer BW
#define				FOC_BW						500.f
#define 			MC_BW						40.f
#define 			MOBS_BW 				    1.2f*MC_BW

//Motor             A Constants
#define             C_P_A						3.f             //Number of pole pairs for motor A
#define             C_KT_A						0.30465f	  	//Kt = (3*P/4)*lambda_ds
#define             C_LAMBDA_PM_A				0.0677f			//Lambda_pm for motor A
#define             C_RS_A						0.9f			//Stator resistance for motor A
#define             C_LS_A 						0.002f
#define             INV_C_P_A					0.33333333333333f;	// inverse of pole pairs			

//Motor B Constants
#define             C_P_B						2.f//2.f        //Number of pole pairs  for motor B
#define             C_Polepair_B				2.f//2.f        //Number of pole pairs  for motor B
#define             C_LAMBDA_PM_B				0.13f			//Lambda_pm  
#define             C_KT_B						0.39f			//0.195f //1.5f*C_P_B * C_LAMBDA_PM_B//0.3753f	  	

#define             C_RS_B						1.35f			//Stator resistance for motor B 

#define             C_LD_B						0.008f			// d-axis inductance
#define             C_LQ_B						0.020f		// q-axis inductance

//Motor A and B Constants
#define             C_J							3.2e-4f 			    //Combined motors inertia
#define             C_BP						2.2e-4f
#define             C_T_MU						0.05f
#define             C_THETA_DIFF				-1.067f	
// #define             C_THETA_DIFF				-1.044f		

//DQ transformation Constants
#define             C_COS_PI_O_3 				0.5f
#define             C_SIN_PI_O_3 				0.86602540378444f
#define             C_2_O_3 					0.66666666666667f
#define             C_3_O_2 					1.5f
#define             C_2PI 						6.28318530717959f
#define             C_PI 						3.14159265358979f
#define             C_PI_O_2 					1.57079632679490f
#define             C_PI_O_3 					1.04719755119659f
#define             C_sqrt_3					1.7320508075688f
#define             C_1_O_3						0.3333333333333f
#define             C_1_O_SQ_3  				0.57735f		// 1/sqrt(3)
#define             C_PIO2 						3.14159265358979f/2.f
#define             C_COSWCTS2					1.9021f
#define             C_2sinwcTs_o_wc				1.9673e-4f

//PI                current controller constants
#define             C_SAMPLE_TIME 				0.0001f	   			 //10 kHz current regulator
#define             C_MOTION_SAMPLE_TIME 		0.0001f
#define             C_MOTION_SAMPLE_TIME_SQ 	0.00000001f

#define             C_K_P_D_A 					9.0f				 //1 kHz BW
#define             C_K_P_Q_A 					9.0f				 //1 kHz BW
#define             C_K_I_D_A 					3572.f			     //Place zero over pole
#define             C_K_I_Q_A 					3572.f				 //Place zero over pole


/// My CR Tunning Values

#define             C_K_P_D_B 					21.3863f//15.f//21.3863f			 //15.f				 //500 Hz BW 
#define             C_K_I_D_B 					3639.6f//2028.5f//2028.5f//3639.6f				 //2028.5f			     //Place zero over pole 
#define             C_K_P_Q_B 					53.7377f//40.f//40.f//53.7377f			 //40.f				 //500 Hz BW 
#define             C_K_I_Q_B 					3639.6f//2878.f//2878.f//3639.6f				//2878.f			     //Place zero over pole 
                               



#define             C_B_A 						0.0265f				// 20 Hz
#define             C_K_A 						0.6658f				// 4 Hz
#define             C_K_IA						2.7f				// 0.8 Hz

#define             C_R_O_DR					38.9078f// 40 Hz   // 19.62f				//500 Hz
#define             C_R_IO_DR					22920.f //		400 Hz //	5895.9f	//50  Hz
#define             C_R_RO_DR					0.8754f// Kico*2*sin(wc*Ts)/wc

#define             C_R_O_QR					98.7873f// 40Hz   //63.6f				//500 Hz
#define             C_R_IO_QR					57012.f//	400Hz //	18402.8f	//50  Hz 
#define             C_R_RO_QR					2.1775f//Kico*2*sin(wc*Ts)/wc

//d-axis
#define             C_R_O_DF					2.267e3f  //372.6f*8.f							
#define             C_R_IO_DF					5.5625e5f //19371.6f*8.f

//q-axis
#define             C_R_O_QF					2.356e3f    //372.6f*8.f							
#define             C_R_IO_QF					5.5343e5f	//19371.6f*8.f		


//Encoder
#define             C_CPR						8000.f	    			//Counts per revolution (2000 lines)
#define             C_RAD_PER_COUNT				7.853981633974e-4f  	//Radians per count

//Protection variables
#define             C_WM						200.f			// Overspeed Limit
#define             C_OC_D                  	5.f             //Current Protection for both IPMSM and SPMSM in d-axis
#define             C_OC_A						9.f		    	//Over current protection in inverter A
#define             C_OC_B						10.f//15.f			//Over current protection in inverter B
#define             C_OV						350.f			//Over voltage protection for DC link voltage (both A and B)
#define             C_UV_A						40.f			//Under voltage protection for DC link voltage (both A and B)
#define             C_UV_B						40.f			//Under voltage protection for DC link voltage (both A and B)
#define             C_UV                    	0.f
#define             C_BRAKE_ON					350.f			//If DC link is above this voltage the brake IGBT is applied and the initial charge IGBT is turned off

//Error codes
#define             C_FT_OC_IAS					0x1   					//Over current in A phase
#define             C_FT_OC_IBS					0x2	    				//Over current in B phase
#define             C_FT_OC_ICS					0x4						//Over current in C phase
#define             C_FT_OV						0x8		    			//Over voltage
#define             C_FT_UV						0x10			 		//Under voltage
#define             C_FT_EP						0x20		   			//Excessive Position Error
#define             C_FT_WM						0x40					//Over speed limit
//#define           C_FT_UF						0x80					//Under flux limit reached  (F. Quattrone Jan 2011)





