/*************************************************************************/
/*						IPM Control Program - ethernet.h              					   */
/*						v1.0											                                 */
/*						03/27/10											                             */
/*						Wei Xu							                                       */
/*************************************************************************/
/* v1.1  First Version   Implemented based on Jae Suk's code             */
/*        03/27/10                                                       */
/* v1.2  Add command for high frequency injection                        */
/*                                                                       */
/*************************************************************************/

#ifndef ETHERNET_H
#define ETHERNET_H

#include "constants.h"
#ifdef _DATALINK
	#include "XCSDataLink.h"
#endif



//------Datalink Protocol variables------// 

	struct Command { 
	    unsigned ID; 
	    unsigned Data; 
	};
	

	union UnionData { 
	    unsigned uint; 
	    float f; 
	};

	// DataLink Commands are defined here:
	enum { 
		CMD_NOP 	  	  = 1,
		CMD_DISC		  = 2,
	    CMD_STOP          = 10,
	    CMD_STARTUP       = 11,
	    CMD_START         = 12,
	    CMD_REC           = 13,
	    CMD_DATASET       = 14,
	    CMD_SEND_DATA     = 15,
	    CMD_SEND_DATA_SIZE= 16,
	    CMD_SEND_NUM_CHAN = 17,
		//Variable Change
		CMD_TORQUE        = 101,
		CMD_SPEED         = 102,
		CMD_LAMBDA        = 103,
		CMD_MINCHRIP      = 104,     
		CMD_MAXCHRIP	  = 105,
		CMD_WC			  = 106		
	};
	
	// Constants defining the data logging space
	#ifdef _Chirp_Gen
		const int num_channels      = 2;//12;
	#else
		const int num_channels      = 6;//12;
	#endif	
	const int num_samples       = C_SAMPLES/((float)num_channels);

	// DataLink Variables
	#ifdef _DATALINK
		extern DataLink::XCSDataLink link;
		extern Command command;
		extern UnionData data;
		extern float Value_dump[num_channels][num_samples];
		extern bool experiment;
	#endif
#endif /* ETHERNET_H */
