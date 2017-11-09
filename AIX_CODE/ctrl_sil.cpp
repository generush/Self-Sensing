/* Copyright 2003-2004 The MathWorks, Inc. */

// *******************************************************************
// **** To build this mex function use: mex sfun_cppcount_cpp.cpp ****
// *******************************************************************
#include "mex.h"

#include "ctrl_sil.h"
#include "IPMSM.cpp"
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  ctrl_sil

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, outputs, states, etc.).
float theta_sim = 0.f;
float Ias_A_sim  = 0.f , Ibs_A_sim  = 0.f , Ics_A_sim  = 0.f;
float Ias_B_sim  = 0.f, Ibs_B_sim  = 0.f, Ics_B_sim = 0.f;
float INV_Vdc_A_sim    = 0.f;
float INV_Vdc_B_sim    = 0.f;
float Vdc_A_fil_sim    = 0.f;							//Filtered dc link voltage
float Vdc_B_fil_sim    = 0.f;							//Filtered dc link voltage
bool  mj_time = 1.f;       
        
static void mdlInitializeSizes(SimStruct *S)
{
    // No expected parameters
    #define NUM_PARAM 3
    ssSetNumSFcnParams(S, NUM_PARAM);
	ssSetSFcnParamTunable(S,0,true);   /* Tunable  wr_set*/
    // Parameter mismatch will be reported by Simulink
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // Specify I/O
    #define NUM_IN_PORT 4
    #define NUM_OUT_PORT 5
    if (!ssSetNumInputPorts(S, NUM_IN_PORT)) return;
    for(int i = 0; i < NUM_IN_PORT; i++)
    {
        
        if(i == 1 || i == 2)
        {
            ssSetInputPortWidth(S, i, 3);
            ssSetInputPortDirectFeedThrough(S, i, 1);
        }
        else
        {
            ssSetInputPortWidth(S, i, 1);
            ssSetInputPortDirectFeedThrough(S, i, 1);
        }
        
    }
    if (!ssSetNumOutputPorts(S,NUM_OUT_PORT)) return;
    for(int i = 0; i < NUM_OUT_PORT; i++)
    {
        //ssSetOutputPortWidth(S, i, DYNAMICALLY_SIZED);
        ssSetOutputPortWidth(S, i, 4);
        ssSetOutputPortWidth(S, i, 4);
		
		if(i == 4)
		{
			ssSetOutputPortWidth(S, i, 1);
			ssSetOutputPortWidth(S, i, 1);
		}
		
    }
	ssSetOutputPortWidth(S, 3, 6);
    ssSetOutputPortWidth(S, 3, 6);
    

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE);
				 
	ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetOffsetTime(S, 0, 0.0);
	ssSetSampleTime(S, 0, 5e-5);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Store new C++ object in the pointers vector
    DoubleAdder *da  = new DoubleAdder();
    ssGetPWork(S)[0] = da;
	Wrm_Count = 0.f;
	w_rm_com = 0.f;

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
			
	//Clear DB-DTFC commands
	#ifdef _DBDTFC_B 
		DBDTFC_B.clear();
	#endif

	T_com_A = 0.f;
	T_com_B = 0.f;
			

	w_rm_left_kp1 = 0.f;
	w_rm_hat_kp1 = 0.f;
	theta_rm_hat_kp1_A =0.f;
	theta_rm_hat_B = 0.f;
	k_hfi                          =  0;//1;//
	Flag_fault_A = 0;Flag_fault_B = 0;
	
	
    InputRealPtrsType  u0 = ssGetInputPortRealSignalPtrs(S,0);
    theta_sim = *u0[0];
    
    InputRealPtrsType  u1 = ssGetInputPortRealSignalPtrs(S,1);
    Ias_A_sim = *u1[0];
    Ibs_A_sim = *u1[1];
    Ics_A_sim = *u1[2];
    
    InputRealPtrsType  u2 = ssGetInputPortRealSignalPtrs(S,2);
    Ias_B_sim = *u2[0];
    Ibs_B_sim = *u2[1];
    Ics_B_sim = *u2[2];
    
    InputRealPtrsType  u3 = ssGetInputPortRealSignalPtrs(S,3);
    Vdc_A_fil_sim = *u3[0];
    Vdc_B_fil_sim = *u3[0];
    INV_Vdc_A_sim = 1/Vdc_A_fil_sim;
    INV_Vdc_B_sim = 1/Vdc_B_fil_sim;
    	
	Flag_run = 1;
	Flag_align = 0;
	for(int j = 0; j < 1000; j++) {control_ISR();}
	Flag_run = 1;

}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Retrieve C++ object from the pointers vector
    //DoubleAdder *da = static_cast<DoubleAdder *>(ssGetPWork(S)[0]);
   
    
    // Get data addresses of I/O
    InputRealPtrsType  u0 = ssGetInputPortRealSignalPtrs(S,0);
    theta_sim = *u0[0];
    
    InputRealPtrsType  u1 = ssGetInputPortRealSignalPtrs(S,1);
    Ias_A_sim = *u1[0];
    Ibs_A_sim = *u1[1];
    Ics_A_sim = *u1[2];
    
    InputRealPtrsType  u2 = ssGetInputPortRealSignalPtrs(S,2);
    Ias_B_sim = *u2[0];
    Ibs_B_sim = *u2[1];
    Ics_B_sim = *u2[2];
    
	InputRealPtrsType  u3 = ssGetInputPortRealSignalPtrs(S,3);
    Vdc_A_fil_sim = *u3[0];
    Vdc_B_fil_sim = *u3[0];
    INV_Vdc_A_sim = 1/Vdc_A_fil_sim;
    INV_Vdc_B_sim = 1/Vdc_B_fil_sim;
    
    real_T *y0 = ssGetOutputPortRealSignal(S, 0); 
    real_T *y1 = ssGetOutputPortRealSignal(S, 1);
	real_T *y2 = ssGetOutputPortRealSignal(S, 2);
	real_T *y3 = ssGetOutputPortRealSignal(S, 3);
	real_T *y4 = ssGetOutputPortRealSignal(S, 4);

	
	w_rm_set 	= (real_T)mxGetScalar(ssGetSFcnParam(S,0));
    T_set_B 	= (real_T)mxGetScalar(ssGetSFcnParam(S,1));
    lambda_set 	= (real_T)mxGetScalar(ssGetSFcnParam(S,2));
	
	if(mj_time){
		if(Flag_fault_A|Flag_fault_B)
		{
			mexPrintf("Motor Faulted: A: %#010x \t B: %#010x\n",Flag_fault_A,Flag_fault_B );
			Flag_fault_A = 0;
			Flag_fault_B = 0;
		}
		control_ISR();
		mj_time = 0.f;
	}

	
	
    y0[0] = Uout_A[0];
    y0[1] = Uout_A[1] - 0.5;
    y0[2] = Uout_A[2] - 0.5;
    y0[3] = Uout_A[3] - 0.5;
    
    y1[0] = Uout_B[0];
    y1[1] = Uout_B[1] - 0.5;
    y1[2] = Uout_B[2] - 0.5;
    y1[3] = Uout_B[3] - 0.5;

	y2[0] = I_dq_f_B.get_d_comp();// 
    y2[1] = I_dq_f_B.get_q_comp();

	y3[0] = theta_r_hats_B;
    y3[1] = T_com_B;//T_e_hat_kp1;//T_com;
    y3[2] = T_com_A;//DBDTFC_B.theta_i;
    y3[3] = T_e_hat_kp1;//SO.theta_err_hat;
	y3[4] = 0.f; 
    y3[5] = I_dq_c_B.get_d_comp();;
	
	y4[0] = T_com_A;
}

#define MDL_UPDATE 
#if defined(MDL_UPDATE) && defined(MATLAB_MEX_FILE) 
static void mdlUpdate(SimStruct *S, int_T tid) 
{
		mj_time = 1.f;

} 
#endif 


/* Define to indicate that this S-Function has the mdlG[S]etSimState mothods */
#define MDL_SIM_STATE

/* Function: mdlGetSimState =====================================================
 * Abstract:
 *
 */
static mxArray* mdlGetSimState(SimStruct* S)
{
    // Retrieve C++ object from the pointers vector
    DoubleAdder *da = static_cast<DoubleAdder*>(ssGetPWork(S)[0]);
    return mxCreateDoubleScalar(da->GetPeak());
}
/* Function: mdlGetSimState =====================================================
 * Abstract:
 *
 */
static void mdlSetSimState(SimStruct* S, const mxArray* ma)
{
    // Retrieve C++ object from the pointers vector
    DoubleAdder *da = static_cast<DoubleAdder*>(ssGetPWork(S)[0]);
    da->SetPeak(mxGetPr(ma)[0]);
}

// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S)
{
    // Retrieve and destroy C++ object
    DoubleAdder *da = static_cast<DoubleAdder *>(ssGetPWork(S)[0]);
    delete da;
}


// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
