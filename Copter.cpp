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
	 rc_futaba_init();
	 IMU_6050_init();





	/*
	 * 自驾仪导航控制算法初始化，自驾仪如果做纯软件仿真的话
	 * 本身自成一体，不需要外部输入
	 */
	rc_in_init();		// sets up rc channels from radio
	rc_out_init();		// sets up the timer libs

	parameter_init();	//一些全局参数初始化
	controller_init(); 	// 增稳模式下pid控制器参数初始化

	roll_pitch_mode=ROLL_PITCH_STABLE;
	yaw_mode=YAW_STABILE;

	cos_roll_x 	= 1;
	cos_pitch_x 	= 1;
	cos_yaw_x 		= 1;
	sin_pitch_y=0;
	sin_yaw_y=0;
	sin_roll_y=0;

	g.channel_throttle.control_in=500;
	g.channel_throttle.servo_out=500;
	g.throttle_cruise=500;
}

void Copter::update_current_flight_mode(void)
{
#if 0
	if(g.rc_5.radio_in>1000&&g.rc_5.radio_in<1500)
	{
		control_mode=ACRO;
	}
	else if(g.rc_5.radio_in>1500 && g.rc_5.radio_in<1900)
	{
		control_mode=STABILIZE;
		//DEBUG_PRINTF("飞控模式是增稳模式\n");
	}
	else if(g.rc_5.radio_in>1900)
	{
		control_mode=AUTO;
	}
#endif

	int8_t switch_position;
	uint16_t mode_in;

	mode_in = g.rc_5.radio_in;

	if      (mode_in < 1231) switch_position = 0;
	else if (mode_in < 1361) switch_position = 1;
	else if (mode_in < 1491) switch_position = 2;
	else if (mode_in < 1621) switch_position = 3;
	else if (mode_in < 1750) switch_position = 4;
	else switch_position = 5;

	control_mode = switch_position;

	/*
	 * 根据飞行模式决定控制yaw roll pitch throttle的模式
	 */
	switch(control_mode)
	{
	case ACRO:
		yaw_mode                 = YAW_ACRO;
		roll_pitch_mode 		= ROLL_PITCH_ACRO;
		throttle_mode   		= THROTTLE_MANUAL;
		break;
	case STABILIZE:
		DEBUG_PRINTF("飞控模式是增稳模式\n");
		yaw_mode                 = YAW_HOLD;
		roll_pitch_mode 		= ROLL_PITCH_STABLE;
		throttle_mode   		= THROTTLE_MANUAL;
		break;

	default:
		break;
	}
}

void Copter::parameter_init()
{

}

void Copter::controller_init()
{
	//姿态角度控制--第1级pid参数设置
	float stabilize_roll_p=3.69;
	float stabilize_roll_i=0.0;
	float stabilize_roll_d=0.0;
	g.pi_stabilize_roll.set_kP(stabilize_roll_p);
	g.pi_stabilize_roll.set_kI(stabilize_roll_i);
	g.pi_stabilize_roll.set_kD(stabilize_roll_d);

	float stabilize_pitch_p=3.69;
	float stabilize_pitch_i=0.0;
	float stabilize_pitch_d=0.0;
	g.pi_stabilize_pitch.set_kP(stabilize_pitch_p);
	g.pi_stabilize_pitch.set_kI(stabilize_pitch_i);
	g.pi_stabilize_pitch.set_kD(stabilize_pitch_d);

	float stabilize_yaw_p=4.0;
	float stabilize_yaw_i=0.0;
	float stabilize_yaw_d=0.0;
	g.pi_stabilize_yaw.set_kP(stabilize_yaw_p);
	g.pi_stabilize_yaw.set_kI(stabilize_yaw_i);
	g.pi_stabilize_yaw.set_kD(stabilize_yaw_d);

	//姿态角速度控制--第2级pid参数设置
	float stabilize_roll_rate_p=0.15;
	float stabilize_roll_rate_i=0.0;
	float stabilize_roll_rate_d=0.0;
	g.pid_rate_roll.set_kP(stabilize_roll_rate_p);
	g.pid_rate_roll.set_kI(stabilize_roll_rate_i);
	g.pid_rate_roll.set_kD(stabilize_roll_rate_d);

	float stabilize_pitch_rate_p=0.15;
	float stabilize_pitch_rate_i=0.0;
	float stabilize_pitch_rate_d=0.0;
	g.pid_rate_pitch.set_kP(stabilize_pitch_rate_p);
	g.pid_rate_pitch.set_kI(stabilize_pitch_rate_i);
	g.pid_rate_pitch.set_kD(stabilize_pitch_rate_d);

	float stabilize_yaw_rate_p=0.2;
	float stabilize_yaw_rate_i=0.0;
	float stabilize_yaw_rate_d=0.0;
	g.pid_rate_yaw.set_kP(stabilize_yaw_rate_p);
	g.pid_rate_yaw.set_kI(stabilize_yaw_rate_i);
	g.pid_rate_yaw.set_kD(stabilize_yaw_rate_d);

	/*
	 * 油门控高pid参数设置
	 */
	float pid_p_alt=1;
	g.pi_alt_hold.set_kP(pid_p_alt);
	g.pi_alt_hold.set_kI(0.0);
	g.pi_alt_hold.set_kD(0.0);

	float pid_p_throttle=0.5;
	g.pid_throttle.set_kP(pid_p_throttle);
	g.pid_throttle.set_kI(0.0);
	g.pid_throttle.set_kD(0.0);

}

void Copter::update_all_external_device_input( void )
{
	/*
	 * 这个函数其实并不需要
	 */

	/*
	 * 控制数据-遥控器输入
	 */
	all_external_device_input.rc_raw_in_0    =    1500;
	all_external_device_input.rc_raw_in_1    =    1500;
	all_external_device_input.rc_raw_in_2    =    1500;
	all_external_device_input.rc_raw_in_3    =    1500;
	all_external_device_input.rc_raw_in_4    =    1100;
	all_external_device_input.rc_raw_in_5    =    1500;
	all_external_device_input.rc_raw_in_6    =    1500;
	all_external_device_input.rc_raw_in_7    =    1500;
	all_external_device_input.rc_raw_in_8    =    1500;

	/*
	 * 导航数据--GPS数据
	 */
	//all_external_device_input.latitude    =    (fdm_feed_back.latitude *RAD_TO_DEG)*1e7;
	all_external_device_input.latitude    =    39*1e7;
	all_external_device_input.longitude =    116*1e7;
	all_external_device_input.altitude    =   100;
	all_external_device_input.v_north    =    1;
	all_external_device_input.v_east    =    2;
	all_external_device_input.v_down    =    3;

	/*
	 * 导航数据--IMU姿态传感器数据
	 */
	all_external_device_input.accel_x    =    1;
	all_external_device_input.accel_y    =    2;
	all_external_device_input.accel_z    =    3;
	all_external_device_input.gyro_x    =    1;
	all_external_device_input.gyro_y    =    2;
	all_external_device_input.gyro_z    =    3;
}


void Copter::loop_slow()
{
    DEBUG_PRINTF("Hello loop_slow\n");
}

void Copter::end_of_task()
{
	DEBUG_PRINTF("Hello end_of_task\n");
}

