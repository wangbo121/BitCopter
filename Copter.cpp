/*
 * Copter.cpp
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#include "Copter.h"

Copter copter;

Copter::Copter()
{
	/*
	* 在构造函数的开始就初始化一些内部变量
	*/
	control_mode            = STABILIZE;
}

void Copter::setup( void )
{
	/*
	 * 外围设备的初始化
	 */
//	rc_in_init();
//
//	rc_out_init();
//
//	gyro_acc_init();
//
//	magnet_init();
//
//	barometer_init();
//
//	flash_init();
//
//	led_init();

	/*
	 * 导航制导控制的参数初始化
	 */
//	parameter_init();
//
//	controller_init();

	 init_futaba();
}

void Copter::update_current_flight_mode(void)
{
	if(g.rc_5.radio_in>1000&&g.rc_5.radio_in<1500)
	{
		control_mode=ACRO;
	}
	else if(g.rc_5.radio_in>1500 && g.rc_5.radio_in<1900)
	{
		control_mode=STABILIZE;
	//	std::cout<<"飞控模式是增稳模式:"<<std::endl;
	}
	else if(g.rc_5.radio_in>1900)
	{
		control_mode=AUTO;
		//std::cout<<"飞控模式是绕航点飞行:"<<std::endl;
	}

	/*
	 * 根据飞行模式决定控制yaw roll pitch throttle的模式
	 */
	switch(control_mode)
	{
	case ACRO:
		yaw_mode                = YAW_ACRO;
		roll_pitch_mode = ROLL_PITCH_ACRO;
		throttle_mode   = THROTTLE_MANUAL;
		break;
	case STABILIZE:
		yaw_mode                = YAW_HOLD;
		roll_pitch_mode = ROLL_PITCH_STABLE;
		throttle_mode   = THROTTLE_MANUAL;
		break;
	case AUTO:
		yaw_mode                = YAW_AUTO;
		roll_pitch_mode = ROLL_PITCH_AUTO;
		throttle_mode   = THROTTLE_AUTO;

		// loads the commands from where we left off
		//init_commands();
		break;
	case LOITER:
//		yaw_mode                = LOITER_YAW;
//		roll_pitch_mode = LOITER_RP;
//		throttle_mode   = LOITER_THR;
//		set_next_WP(&current_loc);
		break;
	default:
		break;
	}
}
