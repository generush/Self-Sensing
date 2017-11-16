#include"hardware.h"
#include"./shared/LinkPort.h"
#include"./shared/SDSP_to_CDSP_Packet.h"
#include"./shared/CDSP_to_SDSP_Packet.h"

#include"./shared/sizes.h"

#include<terminal.h>

extern SDSP_to_CDSP_Packet CDSP_RX_Buffer;
extern CDSP_to_SDSP_Packet CDSP_TX_Buffer;

extern DMA_Transfer CDSP_RX_DMA_Transfer;
extern DMA_Transfer CDSP_TX_DMA_Transfer;


SDSP_to_CDSP_Packet segment("seg_CommBuffers") CDSP_RX_Buffer = {0,0};
CDSP_to_SDSP_Packet segment("seg_CommBuffers") CDSP_TX_Buffer = {
	0x44332211, // command
	{0x12345687,0x76543210},      // Emuclock

	{  // MonitorVars:
		0x01020304,
		0x11121314,
		0x21222324,
		0x31323334,
		0x41424344,
		0x51525354,
		0x61626364,
		0x71727374,
		0x81828384,
		0x91929394,
		0xA1A2A3A4,
		0xB1B2B3B4,
		0xC1C2C3C4,
		0xD1D2D3D4,
		0xE1E2E3E4,
		0xF1F2F3F4,
		0x01020304,
		0x11121314,
		0x21222324,
		0x31323334,
		0x41424344,
		0x51525354,
		0x61626364,
		0x71727374,
		0x81828384,
		0x91929394,
		0xA1A2A3A4,
		0xB1B2B3B4,
		0xC1C2C3C4,
		0xD1D2D3D4,
		0xE1E2E3E4,
		0xF1F2F3F4
	},

	{1, 2, 3, 4, 5, 6, 7}
};

// configuration data for DMA transfer
DMA_Transfer CDSP_RX_DMA_Transfer = {
	0,0,0, // EC,EM,EI
	0, // GP
	0, // CP
	sizeof48(CDSP_RX_Buffer), // IC
	1,                        // IM
	(SDSP_to_CDSP_Packet*)0x50000 // II
};


DMA_Transfer CDSP_TX_DMA_Transfer = {
	0,0,0, // EC,EM,EI
	0, // GP
	0, // CP
	sizeof48(CDSP_TX_Buffer), // IC
	1,                        // IM
	convert_32_to_48_adress(&CDSP_TX_Buffer) // II
};


void init_communication(){
	CDSP_RX_DMA_Transfer.II = convert_32_to_48_adress(&CDSP_RX_Buffer);
	CDSP_TX_DMA_Transfer.II = convert_32_to_48_adress(&CDSP_TX_Buffer);

	// reset LinkPort to init value
	LP0::disable();
	LP0::reset();

	LP0::init(LP0::eight_bit_bus, LP0::ext_wordsize, 1);
	LP0::receive();

	LP0::disableclear_interrupt();
	LP0::setup_interrupt();

	LP0::setup_dma(CDSP_RX_DMA_Transfer);
	LP0::enable_dma();
	LP0::enable();
}

extern bool service_communication();
