// Encoder.h
// 04/18/15
// Marc Petit
#include "constants.h"
#include "hardware.h"

float norm_angle(float angle);
extern int Flag_init_pos;
extern const int motor_pos;

class Encoder
{
	public:
		float theta_rm_A;
		float theta_r_A;
		float theta_rm_B;
		float theta_r_B;
		Encoder();
		void  update();
		void  clear();
		float encoder_count_offset_SF;
		float encoder_count_offset_ZP;
		float first_zp;
	private:
		float encoder_count;
		float last_encoder_count;
		float ENC_Count;
};



