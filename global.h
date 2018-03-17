/*
 * global.h
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

/*
 * 简单打印调试信息
 */
#define DEBUG_SWITCH   1//如果不想打印信息，就将这句代码注释掉

#ifdef    DEBUG_SWITCH
#define DEBUG_PRINTF(fmt,args...) printf(fmt, ##args)
#else
#define DEBUG_PRINTF(fmt,args...) /*do nothing */
#endif

#define UART_FUTABA "/dev/ttyO1"

// YAW debug
// ---------
#define YAW_STABILE 0
#define YAW_ACRO 1
//#define YAW_HOLD 0
//#define YAW_BRAKE 1
#define YAW_RATE 2

#define YAW_MANUAL 3
// Flight modes
// ------------
#define YAW_HOLD                        0
#define YAW_ACRO                        1
#define YAW_AUTO                        2
#define YAW_LOOK_AT_HOME    		    3
#define YAW_TOY                         4       // THOR This is the Yaw mode





#define ROLL_PITCH_STABLE       0
#define ROLL_PITCH_ACRO         1
#define ROLL_PITCH_AUTO         2
#define ROLL_PITCH_STABLE_OF    3
#define ROLL_PITCH_TOY          4       // THOR This is the Roll and Pitch
                                        // mode
#define ROLL_PITCH_MANUAL 5

#define THROTTLE_STABLE 0
#define THROTTLE_ACRO 1
#define THROTTLE_MANUAL         0
#define THROTTLE_HOLD           1
#define THROTTLE_AUTO           2




#define LOITER_MODE 1
#define WP_MODE 2
#define CIRCLE_MODE 3
#define NO_NAV_MODE 4
#define TOY_MODE 5                      // THOR This mode defines the Virtual
                                        // WP following mode

// definitions for earth frame and body frame
// used to specify frame to rate controllers
#define EARTH_FRAME     0
#define BODY_FRAME      1

// RADIANS
#define RADX100 0.000174532925
#define DEGX100 5729.57795

// DEFAULT PIDS

// roll
#define STABILIZE_ROLL_P 0.70
#define STABILIZE_ROLL_I 0.025
#define STABILIZE_ROLL_D 0.04
#define STABILIZE_ROLL_IMAX 7

//pitch
#define STABILIZE_PITCH_P 0.70
#define STABILIZE_PITCH_I 0.025
#define STABILIZE_PITCH_D 0.04
#define STABILIZE_PITCH_IMAX 7

// yaw stablise
#define STABILIZE_YAW_P  0.7
#define STABILIZE_YAW_I  0.02
#define STABILIZE_YAW_D  0.0

// yaw rate
#define RATE_YAW_P  0.135
#define RATE_YAW_I  0.0
#define RATE_YAW_D  0.0

// throttle
#define THROTTLE_P 0.2
#define THROTTLE_I 0.001
#define THROTTLE_IMAX 100

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define AP_MOTORS_MAX_NUM_MOTORS 12


#endif /* GLOBAL_H_ */
