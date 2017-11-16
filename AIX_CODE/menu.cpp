#include "hardware.h" 
//#include "ethernet.h"
//#include <math.h>
//#include <I2C.h>
#include<xci/xci_send_data.h>
#include<xci/xci_get_status.h>
#include<shared/extract.h>

//extern volatile float    Frequency;
//extern volatile float    phi_B_to_A;
//extern volatile float    Uampl_B;
extern int				Flag_fault_A, Flag_fault_B;


static inline float expfu(const float f_, const unsigned u){
	float f = 1.f;
	for(unsigned i=0; i<u; ++i){
		f *= f_;
	}
	return f;
}

static inline void put(const char c){
	display_putchar(c);
}

/*static inline*/ void put_float(const float f_, const unsigned precomma, const unsigned postcomma, const bool unsgned=false){
	float f = f_ / expfu(10, precomma);

	if(! unsgned){
		if(f > 1.f){
			put('>');
			f = 0.9999999;
		}else if(f >= 0.f){
			put(' ');
		}else if(f > -1.f){
			put('-');
			f = -f;
		}else{
			put('<');
			f = 0.9999999;
		}
	}

	for(unsigned i=0; i<precomma; ++i){
		f *= 10.f;
		const char c = static_cast<char>(f);
		f -= c;
		put('0'+c);
	}
	if(postcomma > 0)
		put('.');
	for(unsigned i=0; i<postcomma; ++i){
		f *= 10.f;
		const char c = static_cast<char>(f);
		f -= c;
		put('0'+c);
	}
}

void delay(float);

char hex_m(const char c){
	return ((c>9)?'A'-10:'0') + c;
}

	
	
void menu_clear_position (void){
	display_gotoxy(0,2);
	display_putchar(' ');
	display_gotoxy(10,2);
	display_putchar(' ');
	display_gotoxy(0,3);
	display_putchar(' ');
	display_gotoxy(10,3);
	display_putchar(' ');
}


void menu_init(){
	display_init(
		false,		// display cursor
		false, 		// blink cursor
		true, 		// true -> Increment DDRAM Address, false -> Decrement DDRAM Address
		false		// true -> Display Shift after write, false -> Cursor Shift after write
		);

	display_clear();
	display_gotohome();

	incenc_set_shftinc(10);
	incenc_set_autores(true);
	incenc_set_autoshft(true);
	incenc_write(0);
	
	/*
	.|01234567890123456789|
	-+--------------------+
	0|XCS2000 Power System|
	1|nA=       Status OK |
	2|F1=ASM    F2=SRM    |
	3|                    |
	-+--------------------+
	*/
	//                 01234567890123456789
	display_putstring("Marc's Code v1.0");
	display_gotoxy(0,1);
	display_putstring(" 4#   5#   Status   ");
	//display_gotoxy(0,2);
	//display_putstring(" F=     Hz d=       ");
	//display_gotoxy(0,3);
	//display_putstring(" A=     %  phi=    ");
	//display_putchar(0xDF);

}





unsigned act_menu_position = 1;
void menu_serve(void){
	char status = 0;

	int chan_dec;
	int inc;
	const float Fincrement = 0.1f;
	const unsigned u = incenc_read(&inc);
	
	if(!(u == 0 && -100 < inc && inc < 100)) inc = 0;

	//Frequency += Fincrement * inc;
	
	// XCI2000 converter error testing
	display_gotoxy(18,1);
	if (msc_shutdown_request() == true){
		display_putstring("ER");
	} else {
		display_putstring("OK");
	}
	xci_get_status(Slot4, &status);
	xci_get_status(Slot5, &status);

		
	// I2C reading digital Slot Inputs and show on display 
	display_gotoxy(3,1);
	if(xci_get_status(Slot4, &status)){
//		display_putchar(hex_m(extract<4,8>(status)));
//		display_putchar(hex_m(extract<0,4>(status)));
		display_putchar(hex_m(extract<4,8>(Flag_fault_A)));			// Display Fault DATA
		display_putchar(hex_m(extract<0,4>(Flag_fault_A)));			// WJLEE
	}else{
		display_putstring("ER");
	}
	

	display_gotoxy(8,1);
	if(xci_get_status(Slot5, &status)){
//		display_putchar(hex_m(extract<4,8>(status)));
//		display_putchar(hex_m(extract<0,4>(status)));
		display_putchar(hex_m(extract<4,8>(Flag_fault_B)));
		display_putchar(hex_m(extract<0,4>(Flag_fault_B)));

	}else{
		display_putstring("ER");
	}

/*
	display_putstring(" 6#");
	if(xci_get_status(Slot5, &status)){
		display_putchar(hex_m(extract<4,8>(status)));
		display_putchar(hex_m(extract<0,4>(status)));
	}else{
		display_putstring("ER");
	}

	display_putstring(" 7#");
	if(xci_get_status(Slot5, &status)){
		display_putchar(hex_m(extract<4,8>(status)));
		display_putchar(hex_m(extract<0,4>(status)));
	}else{
		display_putstring("ER");
	}
*/
	
	// Test for any key pressed and store information		

/*	
	if(key_pressed(KEY_F1)){
		act_menu_position = 1;
		menu_clear_position();
		display_gotoxy(0,2);
		display_putchar('>');
	}
	
	
	if(key_pressed(KEY_F2)){
		act_menu_position = 2;
		menu_clear_position();
		display_gotoxy(10,2);
		display_putchar('>');
	}
	if(key_pressed(KEY_F3)){
		act_menu_position = 3;
		menu_clear_position();
		display_gotoxy(0,3);
		display_putchar('>');
	}
	if(key_pressed(KEY_F4)){
		act_menu_position = 4;
		menu_clear_position();
		display_gotoxy(10,3);
		display_putchar('>');
	}

	// Update selected variable
	switch (act_menu_position){
		case 1:{ 
				Frequency += Fincrement * inc;
				Frequency = lib_clip(Frequency, -75.f, 75.f);
				if (fabsf(Frequency) < 0.1f) Frequency = 0.f;
				
			break;	
		}
		case 2:{
				if (inc !=0) Frequency *= -1;
			break;	
		}
		case 3:{
				Uampl_B += inc * 0.005f;
				Uampl_B  = lib_clip(Uampl_B, 0.f, 0.5f);
			break;	
		}
		case 4:{
				phi_B_to_A += inc * 1.f;
				phi_B_to_A  = lib_clip(phi_B_to_A, -90.f, 90.f);
			break;	
		}
		default:{
			break;	
		}
	}
	display_gotoxy(3,2);
	put_float(fabsf(Frequency),2,1,false);
	
	display_gotoxy(14,2);
	if (Frequency <  0.f) display_putstring("left ");
	if (Frequency >  0.f) display_putstring("right");
	if (Frequency == 0.f) display_putstring("stop ");
		
	display_gotoxy(4,3);
	put_float(200.f * Uampl_B,3,0,true);
			
	display_gotoxy(15,3);
	put_float(phi_B_to_A,3,0,false);
*/
}


 


