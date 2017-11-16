#define INIT_ONCE
#include"hardware.h"
#include"init.h"


/*
rom section("seg_FPGA_block_A") ROM_A;
rom section("seg_FPGA_block_B") ROM_B;
rom section("seg_FPGA_block_C") ROM_C;
rom section("seg_FPGA_block_D") ROM_D;
rom section("seg_FPGA_block_E") ROM_E;
rom section("seg_FPGA_block_F") ROM_F;
rom section("seg_FPGA_block_G") ROM_G;
rom section("seg_FPGA_block_H") ROM_H;
rom section("seg_FPGA_block_I") ROM_I;
rom section("seg_FPGA_block_J") ROM_J;
rom section("seg_FPGA_block_K") ROM_K;
rom section("seg_FPGA_block_L") ROM_L;
rom section("seg_FPGA_block_M") ROM_M;
rom section("seg_FPGA_block_N") ROM_N;
rom section("seg_FPGA_block_O") ROM_O;
*/


const AN_T AN_ConverterType[32] =
{
// ANALG_L (topside): 8 x AD1851 (16bit, ±3V) (DA output)
		AD1851, AD1851, AD1851, AD1851,
		AD1851, AD1851, AD1851, AD1851,
// ANALG_M (topside): 8 x AD7894 (14bit, ±10V)(AD input)
		AD7894, AD7894, AD7894, AD7894,
		AD7894, AD7894, AD7894, AD7894,
// ANALG_N (bottomside): 8 x AD7894 14bit, ±10V)(AD input)
		AD7894, AD7894, AD7894, AD7894,
		AD7894, AD7894, AD7894, AD7894,
// ANALG_O (bottomside): N/A
		NOADDA, NOADDA, NOADDA, NOADDA,
		NOADDA, NOADDA, NOADDA, NOADDA
};
