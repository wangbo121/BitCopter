/*
 * Copter.h
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#ifndef COPTER_H_
#define COPTER_H_

//C标准头文件
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>//创建文件
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>

//C++标准头文件
//#include <iostream>
//#include <cmath>

#include "scheduler.h"
#include "global.h"
#include "utility.h"

#include "rc.h"
#include "rc_channel.h"
#include "rc_futaba.h"
#include "IMU.h"
#include "IMU_6050.h"
#include "ahrs_DCM.h"
#include "Parameters.h"
#include "BIT_MATH.h"
#include "all_external_device.h"

//#include "radio.h"
//#include "save_data.h"
//#include "pid.h"
//#include "all_external_device.h"
//#include "SIM_Vehicle.h"
//#include "global.h"
//#include "boatlink.h"
//#include "control.h"
//#include "navigation.h"
//#include "boatlink_udp.h"
//
//#include "udp.h"

/*
 * 控制模式的宏定义
 */
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control  比例控制，其实也就是纯手动控制
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define POSITION 8                      // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10            // Hold a single location using optical flow

class Copter
{
public:
	Copter();

    BIT_Scheduler scheduler;
    static const BIT_Scheduler::Task scheduler_tasks[];

    void setup();
    void loop();

    private:
	AP_RC ap_rc;

	// primary input control channels
	AP_RC_Channel *channel_roll;
	AP_RC_Channel *channel_pitch;
	AP_RC_Channel *channel_throttle;
	AP_RC_Channel *channel_yaw;

	//IMU
	Vector3f omega;
	// Global parameters are all contained within the 'g' class.
	Parameters g;

    uint8_t          control_mode;
    uint32_t        loop_cnt;
	float G_Dt;

	uint8_t yaw_mode;
	// The current desired control scheme for roll and pitch / navigation
	uint8_t roll_pitch_mode;
	// The current desired control scheme for altitude hold
	uint8_t throttle_mode;

	IMU imu;
	//AP_DCM ahrs={imu};
	AP_DCM ahrs;

	////////////////////////////////////////////////////////////////////////////////
	// Rate contoller targets
	////////////////////////////////////////////////////////////////////////////////
	uint8_t rate_targets_frame;    // indicates whether rate targets provided in earth or body frame
	int32_t roll_rate_target_ef;
	int32_t pitch_rate_target_ef;
	int32_t yaw_rate_target_ef;
	int32_t roll_rate_target_bf ;     // body frame roll rate target
	int32_t pitch_rate_target_bf ;    // body frame pitch rate target
	int32_t yaw_rate_target_bf;      // body frame yaw rate target

	////////////////////////////////////////////////////////////////////////////////
	// Orientation
	////////////////////////////////////////////////////////////////////////////////
	// Convienience accessors for commonly used trig functions. These values are generated
	// by the DCM through a few simple equations. They are used throughout the code where cos and sin
	// would normally be used.
	// The cos values are defaulted to 1 to get a decent initial value for a level state
	float cos_roll_x;
	float cos_pitch_x;
	float cos_yaw_x ;
	float sin_yaw_y;
	float sin_roll;
	float sin_pitch;


	float sin_pitch_y;
	//float sin_yaw_y,
	float sin_roll_y;


	// Altitude
	// The cm/s we are moving up or down based on filtered data - Positive = UP
	int16_t climb_rate;
	float target_rangefinder_alt;   // desired altitude in cm above the ground
	int32_t baro_alt;            // barometer altitude in cm above home
	float baro_climbrate;        // barometer climbrate in cm/s

	////////////////////////////////////////////////////////////////////////////////
	// Navigation Yaw control
	////////////////////////////////////////////////////////////////////////////////
	// The Commanded Yaw from the autopilot.
	int32_t nav_yaw;
	// A speed governer for Yaw control - limits the rate the quad can be turned by the autopilot
	int32_t auto_yaw;
	// Used to manage the Yaw hold capabilities -
	bool yaw_stopped;
	uint8_t yaw_timer;
	// Options include: no tracking, point at next wp, or at a target
	//   byte yaw_tracking = MAV_ROI_WPNEXT;
	// In AP Mission scripting we have a fine level of control for Yaw
	// This is our initial angle for relative Yaw movements
	int32_t command_yaw_start;
	// Timer values used to control the speed of Yaw movements
	uint32_t command_yaw_start_time;
	uint16_t command_yaw_time;                                       // how long we are turning
	int32_t command_yaw_end;                                         // what angle are we trying to be
	// how many degrees will we turn
	int32_t command_yaw_delta;
	// Deg/s we should turn
	int16_t command_yaw_speed;
	// Direction we will turn –  1 = CW, 0 or -1 = CCW
	//byte command_yaw_dir;
	uint8_t command_yaw_dir;
	// Direction we will turn – 1 = relative, 0 = Absolute
	uint8_t command_yaw_relative;
	// Yaw will point at this location if yaw_tracking is set to MAV_ROI_LOCATION
	//struct   Location target_WP;
	uint8_t wp_control;//航点的控制方式有悬停，自动，绕圈等
	//uint8_t   yaw_tracking = MAV_ROI_WPNEXT;
	uint8_t   yaw_tracking ;//机头是否朝着航点，还是朝着兴趣点，其实朝着航点就是把兴趣点设置为航点

	// Attitude control variables
	float command_rx_roll=0;        // User commands
	float command_rx_roll_old;
	float command_rx_roll_diff;
	float command_rx_pitch=0;
	float command_rx_pitch_old;
	float command_rx_pitch_diff;
	float command_rx_yaw=0;
	float command_rx_yaw_diff;
	int control_roll;           // PID control results 这个在完全手控情况下直接就是rc_channel的rc_in，所以范围有两种1是-4500～4500 2是0～1000
	int control_pitch; // PID control results 这个在完全手控情况下直接就是rc_channel的rc_in，所以范围有两种1是-4500～4500 2是0～1000
	int control_yaw; // PID control results 这个在完全手控情况下直接就是rc_channel的rc_in，所以范围有两种1是-4500～4500 2是0～1000
	//float K_aux;

	// flight mode specific
	// --------------------
	int8_t takeoff_complete    = true;         // Flag for using gps ground course instead of IMU yaw.  Set false when takeoff command processes.
	int8_t	land_complete;
	long	takeoff_altitude;
	int			landing_distance;					// meters;
	int			landing_pitch;						// pitch for landing set by commands
	int			takeoff_pitch;
	// An additional throttle added to keep the copter at the same altitude when banking
	int16_t angle_boost;

	// Location & Navigation
	int32_t wp_bearing;
	// The location of home in relation to the copter in centi-degrees
	int32_t home_bearing;
	// distance between plane and home in cm
	int32_t home_distance;
	// distance between plane and next waypoint in cm.
	uint32_t wp_distance;//20170919这个貌似有问题,apm2.3 一开始说是厘米,定义的是32位的,但是一会又说// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	//我都按照米来算，嗯，都按照米来算，得到正确的程序

	// GPS variables
	// -------------
	const 	float t7			= 10000000.0;	// used to scale GPS values for EEPROM storage
	float 	scaleLongUp			= 1;			// used to reverse longtitude scaling
	float 	scaleLongDown 		= 1;			// used to reverse longtitude scaling
	int8_t 	ground_start_count	= 5;			// have we achieved first lock and set Home?
	int     ground_start_avg;					// 5 samples to avg speed for ground start
	int8_t ground_start;    					// have we started on the ground?
	bool	GPS_enabled 	= false;			// used to quit "looking" for gps with auto-detect if none present

	// used to manually override throttle in interactive Alt hold modes
	int16_t 	manual_boost;

	bool 		reset_throttle_flag;
	bool 	invalid_throttle;

    // Global parameters are all contained within the 'g' class. 勿删保留
    //Parameters g;

    private:
	//void read_radio();
	void rc_in_init();
	void rc_out_init();
	void parameter_init();
	void controller_init();


	void trim_radio();

	void update_current_flight_mode(void);

	//get_yaw_rate_stabilized_ef(g.rc_4.control_in);
	void get_stabilize_roll(int32_t target_angle);

	void get_stabilize_pitch(int32_t target_angle);
	void get_stabilize_yaw(int32_t target_angle);
	void get_stabilize_rate_yaw(int32_t target_rate);
	void get_acro_roll(int32_t target_rate);
	void get_acro_pitch(int32_t target_rate);
	void get_acro_yaw(int32_t target_rate);
	// Roll with rate input and stabilized in the earth frame
	void get_roll_rate_stabilized_ef(int32_t stick_angle);

	// Pitch with rate input and stabilized in the earth frame
	void get_pitch_rate_stabilized_ef(int32_t stick_angle);

	// Yaw with rate input and stabilized in the earth frame
	void get_yaw_rate_stabilized_ef(int32_t stick_angle);

	// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
	void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );

	// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
	void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );

	// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
	void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );

	// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
	void update_rate_contoller_targets();

	// run roll, pitch and yaw rate controllers and send output to motors
	// targets for these controllers comes from stabilize controllers
	void run_rate_controllers();

	int16_t get_rate_roll(int32_t target_rate);

	int16_t get_rate_pitch(int32_t target_rate);

	int16_t get_rate_yaw(int32_t target_rate);

	int16_t get_throttle_rate(int16_t z_target_speed);

	/*
	*  reset all I integrators
	*/
	void reset_I_all(void);

	void reset_rate_I();

	void reset_throttle_I(void);

	void reset_stability_I(void);

	void update_yaw_mode(void);

	void update_roll_pitch_mode(void);


	void motors_output();
	void update_throttle_mode();

	void read_radio();


	/*
	 * angle boost 角度加速器
	 * 这里的意思应该是俯仰和滚转角导致掉高，所以我们有需要加上这一部分损失的油门量 20170920
	 */
	int16_t get_angle_boost(int16_t value);

	// Keeps old data out of our calculation / logs
	void reset_nav_params(void);


	void adjust_altitude();

	void update_throttle_cruise();

	int16_t get_nav_throttle(int32_t z_error);

	int get_z_damping();


















    void loop_fast();
    void loop_slow();
    void end_of_task();

    void send_ap2gcs_cmd_boatlink();
    void send_ap2gcs_wp_boatlink();
    void send_ap2gcs_realtime_data_boatlink();

    void send_ap2gcs_realtime_data_boatlink_by_udp();

    void record_config();//记录配置文件
    void record_wp();//记录航点文件
    void record_log();//记录日志

    void set_rc_out();//这给用来设置舵机和电机所使用的pwm波，频率是50hz
    void set_gpio();//设置gpio，
    void set_analogs();//设置模拟量
    void set_relays();//设置继电器开关量

    void get_timedata_now();//获取当前的时间，包含年月日时分秒的
    void update_all_external_device_input( void );
    void update_GPS();

//    Watercraft::sitl_input input;//这个是4个电机的输入，然后用于multi_copter.update(input)更新出飞机的飞行状态
//    Watercraft::sitl_fdm fdm;
    uint16_t servos_set_out[4];//这是驾驶仪计算的到的motor_out中的四个电机的转速，给电调的信号，1000～2000

};

extern Copter copter;



#endif /* COPTER_H_ */
