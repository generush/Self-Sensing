/*************************************************************************/
/*						IPM Control Program - ethernet.cpp					               */
/*						v1.0											                                 */
/*						03/27/10											                             */
/*						Wei Xu							                                       */
/*************************************************************************/
/* v1.1  First Version   Implemented based on Jae Suk's code             */
/*        03/27/10                                                       */
/* v1.2  Add command for high frequency injection                        */
/*                                                                       */
/*************************************************************************/

#include "ethernet.h"
//#include "XCSDataLink.h"
//#include <terminal.h>
//#include "hardware.h"


DataLink::XCSDataLink link;

//Data logging variable in SRAM


_Pragma( "section( \"seg_SRAM\" )" )
float Value_dump[num_channels][num_samples];
//_Pragma( "section( \"/NO_INIT seg_SDRAM_NOINIT\" )" )
//Data logging variable in SRAM
//_Pragma( "section( \"/NO_INIT seg_SDRAM_NOINIT\" )" )
//_Pragma( "section( \"seg_SRAM\" )" )
//char Value_names[num_channels][50];

//------- Functions ----------\\


