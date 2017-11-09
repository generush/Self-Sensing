#pragma once
//////#include"XCP2000LIB.h"
#include"constants.h"
#ifdef VDSP
	#include<AD21161.h> 		// search from the installation directory

	/******************/
	/* FPGA-Structure */
	/******************/
	//#define SHOW_HARDWARE_INCLUDE

	/*MSC*/	#include<MSC.h>
	/* A */	#include<PORT_A.h>
	/* B */	#include<PWM_B.h>
	/* C */	#include<PWM_C.h>
	/* D */	#include<PORT_D.h>
	/* E */ #include<PORT_E.h>
	/* F */
	/* G */	//#include<EMPTY_G.h>
	/* H */ #include<DISPL_H.h> 
	/* I */
	/* J */
	/* K */
	/* L */	#include<ANALG_L.h>
	/* M */
	/* N */ #include<ANALG_N.h>
	/* O */
	
	#include<ENCOD_A.h>	
#else
	//#warning "Simulink Hardware"
	struct PWM_sim
	{
		bool CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7; 
	};
	
	template<class T>
	inline T lib_clip(const T t, const T t_min, const T t_max)
	{	
		if( t < t_min) return t_min;
		if( t > t_max) return t_max;
		else return t;
	 }
#endif

/*
#include<ROM.h>
extern rom
	       ROM_A, ROM_B, ROM_C, ROM_D, ROM_E, ROM_F, ROM_G,
	ROM_H, ROM_I, ROM_J, ROM_K, ROM_L, ROM_M, ROM_N, ROM_O;
*/
