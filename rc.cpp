/*
 * rc.cpp
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#include <stdint.h>

#include "rc.h"

#include "all_external_device.h"



AP_RC::AP_RC()
{

}

void
AP_RC::init()
{



	/*
	 * 在我看来初始化，就应该是下面的这些set_ch_pwm呀，
	 * 但是他们为什么用的上面了，是为了封装吗？
	 */
	/*
	set_ch_pwm(0, 1500);
	set_ch_pwm(1, 1500);
	set_ch_pwm(2, 1500);
	set_ch_pwm(3, 1500);
	*/

}

uint16_t
AP_RC::input_ch(uint8_t ch)
{
	/*
	 * 这里是从hal获取通道的pwm值
	 */
	switch(ch)
	{
	case CH_1:
		return (uint16_t)all_external_device_input.rc_raw_in_0;
		//return 1500;
		//return hal.rcin->read(CH_1);
		break;
	case CH_2:
		return (uint16_t)all_external_device_input.rc_raw_in_1;
		//return 1500;//1200;
		//return hal.rcin->read(CH_2);
		break;
	case CH_3:
		return (uint16_t)all_external_device_input.rc_raw_in_2;
		//return 1500;//1300;
		//return hal.rcin->read(CH_3);
		break;
	case CH_4:
		//return 1800;//1700;
		return (uint16_t)all_external_device_input.rc_raw_in_3;
		//return 1500;
		//return hal.rcin->read(CH_4);
		break;
	case CH_5:
		//return 1400;//特技，完全手控模式
		return (uint16_t)all_external_device_input.rc_raw_in_4;
		//return 1700;//增稳
		break;
	case CH_6:
		break;
	case CH_7:
		break;
	case CH_8:
		break;

	default:
		break;
	}
}

void
AP_RC::output_ch_pwm(uint8_t ch, uint16_t pwm)
{

	switch(ch)
	{
	case CH_1:
		all_external_device_output.rc_raw_out_0=(float)pwm;
		break;
	case CH_2:
		all_external_device_output.rc_raw_out_1=(float)pwm;
		break;
	case CH_3:
		all_external_device_output.rc_raw_out_2=(float)pwm;
		break;
	case CH_4:
		all_external_device_output.rc_raw_out_3=(float)pwm;
		break;
	}
}


