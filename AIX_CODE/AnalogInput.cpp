// AnalogInput.cpp
// 04/18/15
// Marc Petit
#include "AnalogInput.h"


AnalogInput::AnalogInput()
{
	Ias_A = 0.f, Ibs_A = 0.f, Ics_A = 0.f;
	Ias_B = 0.f, Ibs_B = 0.f, Ics_B = 0.f;
	I_ds_s_B = 0.f, I_qs_s_B = 0.f;		
	I_ds_s_A = 0.f, I_qs_s_A = 0.f;	
	
	INV_Vdc_A = 0.f;
	INV_Vdc_B = 0.f;

	//-----Measured DC bus link for inverter A-----//
	Vdc_A = 0.f;
	Vdc_A_fil = 0.f;							//Filtered dc link voltage

	//-----Measured DC bus link for inverter B-----//
	Vdc_B = 0.f;
	Vdc_B_fil = 0.f;							//Filtered dc link voltage
	INV_Vdc_B = 0.f;

	V_Hex = 0.f;	

	Offset_cnt = 0;

	Ias_A_Offset = 0.f, Ibs_A_Offset = 0.f;
	Ias_A_Offset_temp = 0.f, Ibs_A_Offset_temp = 0.f;


	Ias_B_Offset = 0.f, Ibs_B_Offset = 0.f;
	Ias_B_Offset_temp = 0.f, Ibs_B_Offset_temp = 0.f;
}
void  AnalogInput::update()
{
	#ifdef VDSP
		//------Read current AD values------//
		Ias_A = CurrentScaleFactor * AN[IA_A] + Ias_A_Offset;
		Ibs_A = CurrentScaleFactor * AN[IB_A] + Ibs_A_Offset;        
		Ics_A = -Ias_A - Ibs_A;

		Ias_B = CurrentScaleFactor * AN[IA_B] + Ias_B_Offset;
		Ibs_B = CurrentScaleFactor * AN[IB_B] + Ibs_B_Offset;        
		Ics_B = -Ias_B - Ibs_B;			

		//------Read DC bus voltages------//
		Vdc_A_fil = C_V_DC_C1*Vdc_A + C_V_DC_C2*Vdc_A_fil;				//Filtered voltage (100 Hz low pass)
		Vdc_A = VoltageScaleFactor_A * (AN[VDC_A] + 0.008723f);	
					
	#else
		//#warning "Simulink Input"
		extern float Ias_A_sim, Ibs_A_sim, Ics_A_sim;
		extern float Ias_B_sim, Ibs_B_sim, Ics_B_sim;
		extern float INV_Vdc_A_sim;
		extern float INV_Vdc_B_sim;
		extern float Vdc_A_fil_sim;							//Filtered dc link voltage
		extern float Vdc_B_fil_sim;							//Filtered dc link voltage
        	
		//------Read current AD values------//
		Ias_A = Ias_A_sim;
		Ibs_A = Ibs_A_sim;        
		Ics_A = Ics_A_sim;

		Ias_B = Ias_B_sim;
		Ibs_B = Ibs_B_sim;        
		Ics_B = Ics_B_sim;			

		//------Read DC bus voltages------//
		Vdc_A_fil = Vdc_A_fil_sim;				//Filtered voltage (100 Hz low pass)
		Vdc_A = Vdc_A_fil_sim;	
	#endif
	
	INV_Vdc_A = 1.f/Vdc_A_fil;

	Vdc_B_fil = Vdc_A_fil;//C_V_DC_C1*Vdc_B + C_V_DC_C2*Vdc_B_fil;				//Filtered voltage (100 Hz low pass)
	//Vdc_B = VoltageScaleFactor_B * (AN[AN9] - 0.066539f); // need to figure out why it doesn't work
	Vdc_B = Vdc_A;

	INV_Vdc_B = 1.f/Vdc_B_fil;  

	V_Hex = C_2_O_3*Vdc_B_fil;

		//---- Offline Calibration -------//	
	if(!Flag_Offset)
	{
			Offset_cnt++;
			Ias_A_Offset_temp -= Ias_A;
			Ibs_A_Offset_temp -= Ibs_A;
			
			Ias_B_Offset_temp -= Ias_B;
			Ibs_B_Offset_temp -= Ibs_B;
							
			if(Offset_cnt == 10000)
			{
				Ias_A_Offset = Ias_A_Offset_temp * 0.0001f;
				Ibs_A_Offset = Ibs_A_Offset_temp * 0.0001f;
			
				Ias_B_Offset = Ias_B_Offset_temp * 0.0001f;
				Ibs_B_Offset = Ibs_B_Offset_temp * 0.0001f;
				
				Flag_Offset = 1;
			}
	}
	/*******************************************************************************************************************/
	/*******************************************************************************************************************/
	//------Converting currents from abc to dq stator reference frame(Inverter A)------//	
	I_ds_s_A = (C_2_O_3)*(Ias_A - Ibs_A*C_COS_PI_O_3 - Ics_A*C_COS_PI_O_3);
	I_qs_s_A = (C_2_O_3)*(Ibs_A*C_SIN_PI_O_3 - Ics_A*C_SIN_PI_O_3);
	//------Converting currents from abc to dq stator reference frame(Inverter B)------//	
	I_ds_s_B = (C_2_O_3)*(Ias_B - Ibs_B*C_COS_PI_O_3 - Ics_B*C_COS_PI_O_3); //Ias_B;
	I_qs_s_B = (C_2_O_3)*(Ibs_B*C_SIN_PI_O_3 - Ics_B*C_SIN_PI_O_3); 

}
