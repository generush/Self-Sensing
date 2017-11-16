// AnalogInput.h
// 04/20/15
// Marc Petit
#pragma once
#include "constants.h"
#include "hardware.h"


extern bool Flag_Offset;
//-----Sensor Scales-----//
static const float	CurrentScaleFactor	= 20.f;	// amps per volt (1:1000 sensor, 50ohm resistor, 1 turns)

static const float	VoltageScaleFactor_A	= 80.72328059f;	// volts per volt (50mA:1200V, 300ohm resistor)
static const float	VoltageScaleFactor_B	= 81.2413681f;	// volts per volt (50mA:1200V, 300ohm resistor)

static const float V_a_inv_n_Scale_Factor_A = 23.53226f;	// [V/V]
static const float V_b_inv_n_Scale_Factor_A = 22.53357f;	// [V/V]
static const float V_c_inv_n_Scale_Factor_A = 23.48325f;	// [V/V]

static const float V_a_inv_n_offset_A = 0.10982f;		// [V]
static const float V_b_inv_n_offset_A = 0.11510f;		// [V]
static const float V_c_inv_n_offset_A = 0.42442f;		// [V]

class AnalogInput
{
	public:
		AnalogInput();
		void  update();
		float Ias_A , Ibs_A , Ics_A;
		float Ias_B , Ibs_B , Ics_B;
		float I_ds_s_B, I_qs_s_B;		
		float I_ds_s_A, I_qs_s_A;	
		
		float INV_Vdc_A;
		float INV_Vdc_B;
		float Vdc_A_fil;							//Filtered dc link voltage
		float Vdc_B_fil;							//Filtered dc link voltage

	private:
		//-----Measured DC bus link for inverter A-----//
		float Vdc_A;
						
		//-----Measured DC bus link for inverter B-----//
		float Vdc_B;

		float V_Hex;	

		int Offset_cnt;

		float Ias_A_Offset , Ibs_A_Offset;
		float Ias_A_Offset_temp , Ibs_A_Offset_temp;


		float Ias_B_Offset , Ibs_B_Offset ;
		float Ias_B_Offset_temp , Ibs_B_Offset_temp;
};



