/*
 * attitude.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#include "Copter.h"

//static int32_t wrap_180(int32_t error)
//{
//    if (error > 18000) error -= 36000;
//    if (error < -18000) error += 36000;
//    return error;
//}

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//get_yaw_rate_stabilized_ef(g.rc_4.control_in);
void
Copter::get_stabilize_roll(int32_t target_angle)
{
	target_angle            = wrap_180(target_angle - ahrs.roll_sensor,100);//20170814这里应该unimode=100
	target_angle             = constrain_value(target_angle, -2000, 2000);

	/*
	 * convert to desired Rate ： 把期望角度转换为期望角速度
	 * 外环，也就是从角度误差经过pid控制器，把控制器的输出作为角速度环的输入，但是外环其实只用了p比例控制，没有用积分和微分
	 */
	int32_t target_rate = g.pi_stabilize_roll.get_p(target_angle);
	//int32_t target_rate = g.pi_stabilize_roll.get_pi(target_angle,G_Dt);//20170919改成pi

	int16_t i_stab;// 增稳模式下的积分量
	if(labs(ahrs.roll_sensor) < 500)
	{
		target_angle            = constrain_value(target_angle, -500, 500);
		i_stab                          = g.pi_stabilize_roll.get_i(target_angle, G_Dt);
	}
	else
	{
		i_stab                          = g.pi_stabilize_roll.get_integrator();
	}

	/*
	 * set targets for rate controller
	 * 这个用EARTH_FRAME还是其他的，有讲究，得注意
	 */
	set_roll_rate_target(target_rate+i_stab, EARTH_FRAME);
}


void
Copter::get_stabilize_pitch(int32_t target_angle)
{
	DEBUG_PRINTF("get_stabilize_pitch    :    pitch target_angle = %u\n",target_angle);

	// angle error
	target_angle = wrap_180(target_angle - ahrs.pitch_sensor,100);

	target_angle = constrain_value(target_angle, -2000, 2000);

	DEBUG_PRINTF("get_stabilize_pitch    :    ahrs.pitch_sensor = %d\n",ahrs.pitch_sensor);
	DEBUG_PRINTF("get_stabilize_pitch    :    wrap target_angle = %d\n",target_angle);

	// convert to desired Rate:
	int32_t target_rate = g.pi_stabilize_pitch.get_p(target_angle);

	DEBUG_PRINTF("get_stabilize_pitch    :    target_rate = %d\n",target_rate);

	int16_t i_stab;
	if(labs(ahrs.pitch_sensor) < 500)
	{
		target_angle = constrain_value(target_angle, -500, 500);
		i_stab             = g.pi_stabilize_pitch.get_i(target_angle, G_Dt);
	}
	else
	{
		i_stab             = g.pi_stabilize_pitch.get_integrator();
	}

	// set targets for rate controller
	set_pitch_rate_target(target_rate + i_stab, EARTH_FRAME);
}

void
Copter::get_stabilize_yaw(int32_t target_angle)
{
	int32_t target_rate,i_term;
	int32_t angle_error;

	// angle error
	angle_error             = wrap_180(target_angle - ahrs.yaw_sensor,100.0);

	angle_error             = constrain_value(angle_error, -4000, 4000);

	// convert angle error to desired Rate:
	target_rate 			 = g.pi_stabilize_yaw.get_p(angle_error);
	i_term 					 = g.pi_stabilize_yaw.get_i(angle_error, G_Dt);

	// set targets for rate controller
	set_yaw_rate_target(target_rate+i_term, EARTH_FRAME);
}

void
Copter::get_stabilize_rate_yaw(int32_t target_rate)
{

}

void
Copter::get_acro_roll(int32_t target_rate)
{

}

void
Copter::get_acro_pitch(int32_t target_rate)
{

}

void
Copter::get_acro_yaw(int32_t target_rate)
{

}

// Roll with rate input and stabilized in the earth frame
void
Copter::get_roll_rate_stabilized_ef(int32_t stick_angle)
{

}

// Pitch with rate input and stabilized in the earth frame
void
Copter::get_pitch_rate_stabilized_ef(int32_t stick_angle)
{

}

// Yaw with rate input and stabilized in the earth frame
void
Copter::get_yaw_rate_stabilized_ef(int32_t stick_angle)
{


}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void
Copter::set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{
	rate_targets_frame = earth_or_body_frame;
	if( earth_or_body_frame == BODY_FRAME )
	{
		roll_rate_target_bf = desired_rate;
	}
	else
	{
		//20170814 我用的是earth_frame，这个要注意，因为后面是要用到的
		roll_rate_target_ef = desired_rate;
	}
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void
Copter::set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{
	 rate_targets_frame = earth_or_body_frame;
	    if( earth_or_body_frame == BODY_FRAME )
	    {
	        pitch_rate_target_bf = desired_rate;
	    }
	    else
	    {
	        pitch_rate_target_ef = desired_rate;
	    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void
Copter::set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{
	//uint8_t rate_targets_frame;
	rate_targets_frame = earth_or_body_frame;
	if( earth_or_body_frame == BODY_FRAME )
	{
		yaw_rate_target_bf = desired_rate;
	}
	else
	{
		yaw_rate_target_ef = desired_rate;
	}
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
Copter::update_rate_contoller_targets()
{
	Vector2f yawvector;
	Matrix3f temp   = ahrs.get_dcm_matrix();

//	std::cout<<"temp.a.x="<<temp.a.x<<"  temp.a.y="<<temp.a.y<<"  temp.a.z="<<temp.a.z<<std::endl;
//	std::cout<<"temp.b.x="<<temp.b.x<<"  temp.b.y="<<temp.b.y<<"  temp.b.z="<<temp.b.z<<std::endl;
//	std::cout<<"temp.c.x="<<temp.c.x<<"  temp.c.y="<<temp.c.y<<"  temp.c.z="<<temp.c.z<<std::endl;

	yawvector.x    = temp.a.x;     // sin
	yawvector.y    = temp.b.x;         // cos
	yawvector.normalize();

	cos_pitch_x      = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
	cos_roll_x         = temp.c.z / cos_pitch_x;                       // level = 1

	cos_pitch_x = constrain_value(cos_pitch_x, 0.0f, 1.0f);
	// this relies on constrain() of infinity doing the right thing,
	// which it does do in avr-libc
	cos_roll_x  		  = constrain_value(cos_roll_x, -1.0f, 1.0f);

	sin_yaw_y        = yawvector.x;                                              // 1y = north
	cos_yaw_x       = yawvector.y;                                              // 0x = north

	// added to convert earth frame to body frame for rate controllers
	sin_pitch          = -temp.c.x;
	sin_roll             = temp.c.y / cos_pitch_x;

	if( rate_targets_frame == EARTH_FRAME )
	{
		// convert earth frame rates to body frame rates
		roll_rate_target_bf       = roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
		pitch_rate_target_bf    = cos_roll_x * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
		yaw_rate_target_bf     = cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
	}
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
Copter::run_rate_controllers()
{
    // call rate controllers
    g.channel_roll.servo_out = get_rate_roll(roll_rate_target_bf);
    g.channel_pitch.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.channel_rudder.servo_out = get_rate_yaw(yaw_rate_target_bf);
}

int16_t
Copter::get_rate_roll(int32_t target_rate)
{
	int32_t p,i,d;                                                                      // used to capture pid values for logging
	int32_t current_rate;                                                       // this iteration's rate
	int32_t rate_error;                                                                 // simply target_rate - current_rate
	int32_t output;                                                                     // output from pid controller

	// get current rate
	current_rate    = (omega.x * DEGX100);

	// call pid controller
	rate_error      = target_rate - current_rate;
	p                      = g.pid_rate_roll.get_p(rate_error);

	i                       = g.pid_rate_roll.get_i(rate_error, G_Dt);
	d                      = g.pid_rate_roll.get_d(rate_error, G_Dt);
	output            = p + i + d;

	// constrain output
	output = constrain_value(output, -5000, 5000);

	// output control
	return output;
}

int16_t
Copter::get_rate_pitch(int32_t target_rate)
{
	int32_t p,i,d;                                                                      // used to capture pid values for logging
	int32_t current_rate;                                                       // this iteration's rate
	int32_t rate_error;                                                                 // simply target_rate - current_rate
	int32_t output;                                                                     // output from pid controller

	// get current rate
	current_rate    = (omega.y * DEGX100);

	// call pid controller
	rate_error      = target_rate - current_rate;
	p                       = g.pid_rate_pitch.get_p(rate_error);
	i                         = g.pid_rate_pitch.get_i(rate_error, G_Dt);
	d                       = g.pid_rate_pitch.get_d(rate_error, G_Dt);
	output              =      p + i + d;

	// constrain output
	output = constrain_value(output, -5000, 5000);

	// output control
	return output;
}

int16_t
Copter::get_rate_yaw(int32_t target_rate)
{
	int32_t p,i,d;                                                                      // used to capture pid values for logging
	int32_t rate_error;
	int32_t output;

	// rate control
	rate_error              = target_rate - (omega.z * DEGX100);

	// separately calculate p, i, d values for logging
	p = g.pid_rate_yaw.get_p(rate_error);
	i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
	d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

	output  = p+i+d;
	output = constrain_value(output, -4500, 4500);

	return  output;
}

int16_t
Copter::get_throttle_rate(int16_t z_target_speed)
{
	int32_t p,i,d;      // used to capture pid values for logging
	int16_t z_rate_error, output;

	z_rate_error    = z_target_speed - climb_rate;              // calc the speed error


	int32_t tmp     = (z_target_speed * z_target_speed * (int32_t)g.throttle_cruise) / 200000;

	if(z_target_speed < 0) tmp = -tmp;

	output                  = constrain_value(tmp, -3200, 3200);

	// separately calculate p, i, d values for logging
	p = g.pid_throttle.get_p(z_rate_error);
	// freeze I term if we've breached throttle limits

	i = g.pid_throttle.get_integrator();

	i = g.pid_throttle.get_i(z_rate_error, .02);

	d = g.pid_throttle.get_d(z_rate_error, .02);

	//
	// limit the rate
	output +=  constrain_value(p+i+d, -80, 120);



	return output;

}


/*
 *  reset all I integrators
 */
void
Copter::reset_I_all(void)
{

}

void
Copter::reset_rate_I()
{

}

void
Copter::reset_throttle_I(void)
{

	// For Altitude Hold
	g.pi_alt_hold.reset_I();
	g.pid_throttle.reset_I();
}

void
Copter::reset_stability_I(void)
{
}

void
Copter::update_yaw_mode(void)
{
	switch(yaw_mode)
	{
	case YAW_STABILE:
		get_stabilize_yaw(g.channel_rudder.control_in);
		break;
	case YAW_ACRO:
		break;

	default:
		break;
	}
}

void
Copter::update_roll_pitch_mode(void)
{
	switch(roll_pitch_mode)
	{
	case ROLL_PITCH_STABLE:
		control_roll            = g.channel_roll.control_in;
		control_pitch           = g.channel_pitch.control_in;
		get_stabilize_roll(control_roll);
		get_stabilize_pitch(control_pitch);
		break;
	case ROLL_PITCH_ACRO:
		get_roll_rate_stabilized_ef(g.channel_roll.control_in);
		get_pitch_rate_stabilized_ef(g.channel_pitch.control_in);
		/* 下面的这两个貌似是ACRO特技模式，但是也是角速度模式 */
		//get_acro_roll(g.rc_1.control_in);
		//get_acro_pitch(g.rc_2.control_in);
		break;

	default:
		break;
	}
}

#define THROTTLE_FILTER_SIZE 4

// controls all throttle behavior
void Copter::update_throttle_mode(void)
{
	int16_t throttle_out;

	switch(throttle_mode)
	{
		case THROTTLE_MANUAL:
			if (g.channel_throttle.control_in > 0)
			{
					if (control_mode == ACRO)
					{
						g.channel_throttle.servo_out 	= g.channel_throttle.control_in;
					}
					else
					{
						angle_boost 		= get_angle_boost(g.channel_throttle.control_in);
						g.channel_throttle.servo_out 	= g.channel_throttle.control_in + angle_boost;
					}
			}
			else
			{
				// we are on the ground
				takeoff_complete = false;
				g.channel_throttle.servo_out = 0;
			}
			break;
		case THROTTLE_HOLD:
			// allow interactive changing of atitude
			adjust_altitude();
			break;
	}
}

/*
 * angle boost 角度加速器
 * 这里的意思应该是俯仰和滚转角导致掉高，所以我们有需要加上这一部分损失的油门量 20170920
 */
int16_t Copter::get_angle_boost(int16_t value)
{
	 float temp = cos_pitch_x * cos_roll_x;
	temp = constrain_value(temp, .75f, 1.0f);
	return ((float)(value + 80) / temp) - (value + 80);
}


// Keeps old data out of our calculation / logs
void Copter::reset_nav_params(void)
{
//    nav_throttle                    = 0;
//
//    // always start Circle mode at same angle
//    //circle_angle                    = 0;
//
//    // We must be heading to a new WP, so XTrack must be 0
//    crosstrack_error                = 0;
//
//    // Will be set by new command
//    target_bearing                  = 0;
//
//    // Will be set by new command
//    wp_distance                     = 0;
//
//    // Will be set by new command, used by loiter
//    long_error                              = 0;
//    lat_error                               = 0;
//
//    // We want to by default pass WPs
//    slow_wp = false;
//
//    // make sure we stick to Nav yaw on takeoff
//    auto_yaw = nav_yaw;
//
//    // revert to smaller radius set in params
//    waypoint_radius = g.waypoint_radius;
}


void
Copter::adjust_altitude()
{
	if(g.channel_throttle.control_in <= 180){
		// we remove 0 to 100 PWM from hover
		manual_boost = g.channel_throttle.control_in - 180;
		//manual_boost = max(-120, manual_boost);
		update_throttle_cruise();

	}else if  (g.channel_throttle.control_in >= 650){
		// we add 0 to 100 PWM to hover
		manual_boost = g.channel_throttle.control_in - 650;
		update_throttle_cruise();
	}else {
		manual_boost = 0;
	}
}

void
Copter::update_throttle_cruise()
{
	int16_t tmp = g.pi_alt_hold.get_integrator();
	if(tmp != 0){
		g.throttle_cruise += tmp;
		reset_throttle_I();
	}
}

int16_t
Copter::get_nav_throttle(int32_t z_error)
{
	static int16_t old_output = 0;
	int16_t rate_error = 0;
	int16_t output = 0;


	// convert to desired Rate:
	rate_error 		= g.pi_alt_hold.get_p(z_error);
	rate_error 		= constrain(rate_error, -100, 100);

	DEBUG_PRINTF("get_nav_throttle    :    z_error = %d\n",z_error);
	DEBUG_PRINTF("get_nav_throttle    :    rate_error = %d\n",rate_error);

	// limit error to prevent I term wind up
	z_error 		= constrain(z_error, -400, 400);

	// compensates throttle setpoint error for hovering
	int16_t iterm = g.pi_alt_hold.get_i(z_error, .1);

	// calculate rate error
	rate_error 		= rate_error - climb_rate;
	DEBUG_PRINTF("get_nav_throttle    :    climb_rate = %d\n",climb_rate);

	// limit the rate
	output =  constrain(g.pid_throttle.get_pid(rate_error, .1), -160, 180);

	// light filter of output
	output = (old_output + output) / 2;

	// save our output
	old_output  = output;

	DEBUG_PRINTF("get_nav_throttle    :    output = %d\n",output);

	// output control:
	return output + iterm;
}

int Copter::get_z_damping()
{
	return 0;
}
