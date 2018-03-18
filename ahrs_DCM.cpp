/*
 * ahrs_DCM.cpp
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#include <stdio.h>
#include "ahrs_DCM.h"

#define OUTPUTMODE 1				// This is just used for debugging, remove later
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

#define Kp_ROLLPITCH 0.05967 		// .0014 * 418/9.81 Pitch&Roll Drift Correction Proportional Gain
#define Ki_ROLLPITCH 0.00001278		// 0.0000003 * 418/9.81 Pitch&Roll Drift Correction Integrator Gain
#define Kp_YAW 0.8		 			// Yaw Drift Correction Porportional Gain
#define Ki_YAW 0.00004 				// Yaw Drift CorrectionIntegrator Gain

#define SPEEDFILT 300				// centimeters/second
#define ADC_CONSTRAINT 900

void
AP_DCM::update_DCM(float _G_Dt)
{
	_imu.update();
	_gyro_vector 	= _imu.get_gyro();			// Get current values for IMU sensors
	_accel_vector 	= _imu.get_accel();			// Get current values for IMU sensors

	printf("update_DCM    :    %f,    %f,    %f\n ",_imu._accel.x, _imu._accel.x, _imu._accel.x);
	printf("update_DCM    :    %f,    %f,    %f\n ",_imu._gyro.x, _imu._gyro.x, _imu._gyro.x);

	matrix_update(_G_Dt); 	// Integrate the DCM matrix
	normalize();			// Normalize the DCM matrix
	//drift_correction();		// Perform drift correction

	euler_angles();			// Calculate pitch, roll, yaw for stabilization and navigation
}

void
AP_DCM::matrix_update(float _G_Dt)
{
	Matrix3f	update_matrix;
	Matrix3f	temp_matrix;

//	std::cout<<"_gyro_vector.x="<<_gyro_vector.x<<std::endl;
//	std::cout<<"_gyro_vector.y="<<_gyro_vector.y<<std::endl;
//	std::cout<<"_gyro_vector.z="<<_gyro_vector.z<<std::endl;
//
//
//	std::cout<<"_accel_vector.x="<<_accel_vector.x<<std::endl;
//	std::cout<<"_accel_vector.y="<<_accel_vector.y<<std::endl;
//	std::cout<<"_accel_vector.z="<<_accel_vector.z<<std::endl;

	_omega=_gyro_vector;
	_dcm_matrix.rotate(_omega*_G_Dt);
}


/**************************************************/
void
AP_DCM::accel_adjust(void)
{
	Vector3f veloc, temp;

	//veloc.x = _gps.ground_speed / 100;		// We are working with acceleration in m/s^2 units

	// We are working with a modified version of equation 26 as our IMU object reports acceleration in the positive axis direction as positive

	//_accel_vector -= _omega_integ_corr % _veloc;		// Equation 26  This line is giving the compiler a problem so we break it up below
	temp.x = 0;
	temp.y = _omega_integ_corr.z * veloc.x; 			// only computing the non-zero terms
	temp.z = -1.0f * _omega_integ_corr.y * veloc.x;	// After looking at the compiler issue lets remove _veloc and simlify

	_accel_vector -= temp;
}


/*************************************************
Direction Cosine Matrix IMU: Theory
William Premerlani and Paul Bizard

Numerical errors will gradually reduce the orthogonality conditions expressed by equation 5
to approximations rather than identities. In effect, the axes in the two frames of reference no
longer describe a rigid body. Fortunately, numerical error accumulates very slowly, so it is a
simple matter to stay ahead of it.
We call the process of enforcing the orthogonality conditions 襯enormalization?
*/
void
AP_DCM::normalize(void)
{
	float error = 0;
	Vector3f	temporary[3];

	int problem = 0;

	error = _dcm_matrix.a * _dcm_matrix.b; 							// eq.18

	temporary[0] = _dcm_matrix.b;
	temporary[1] = _dcm_matrix.a;
	temporary[0] = _dcm_matrix.a - (temporary[0] * (0.5f * error));		// eq.19
	temporary[1] = _dcm_matrix.b - (temporary[1] * (0.5f * error));		// eq.19

	temporary[2] = temporary[0] % temporary[1];							// c= a x b // eq.20

	_dcm_matrix.a = renorm(temporary[0], problem);
	_dcm_matrix.b = renorm(temporary[1], problem);
	_dcm_matrix.c = renorm(temporary[2], problem);

	if (problem == 1)
	{	// Our solution is blowing up and we will force back to initial condition.	Hope we are not upside down!
		_dcm_matrix.a.x = 1.0f;
		_dcm_matrix.a.y = 0.0f;
		_dcm_matrix.a.z = 0.0f;
		_dcm_matrix.b.x = 0.0f;
		_dcm_matrix.b.y = 1.0f;
		_dcm_matrix.b.z = 0.0f;
		_dcm_matrix.c.x = 0.0f;
		_dcm_matrix.c.y = 0.0f;
		_dcm_matrix.c.z = 1.0f;
	}
}

/**************************************************/
Vector3f
AP_DCM::renorm(Vector3f const &a, int &problem)
{
	float	renorm;

	renorm = a * a;
#if 0
	if (renorm < 1.5625f && renorm > 0.64f) {			// Check if we are OK to use Taylor expansion
		renorm = 0.5 * (3 - renorm);					// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1.0 / sqrt(renorm);
		renorm_sqrt_count++;
	} else {
		problem = 1;
		renorm_blowup_count++;
	}
#else
	//20170818原来是上面的，但是我为了测试fdm和apm姿态解算有啥不一样，改成下面的了
	renorm = 1.0 / sqrt(renorm);
	problem=0;

#endif
	return(a * renorm);
}

/**************************************************/
void
AP_DCM::drift_correction(void)
{
	//Compensation the Roll, Pitch and Yaw drift.
	//float mag_heading_x;
	//float mag_heading_y;
	float error_course;
	float accel_magnitude;
	float accel_weight;
	float integrator_magnitude;
	//static float scaled_omega_P[3];
	//static float scaled_omega_I[3];
	static bool in_motion = false;
	Matrix3f rot_mat;

	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	accel_magnitude = _accel_vector.length() / 9.80665f;

	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	accel_weight = dcm_constrain(1 - 2 * fabs(1 - accel_magnitude), 0, 1);	//

	//	We monitor the amount that the accelerometer based drift correction is deweighted for performance reporting
	_health = dcm_constrain(_health+(0.02 * (accel_weight - .5)), 0, 1);

	// adjust the ground of reference
	_error_roll_pitch =  _dcm_matrix.c % _accel_vector;			// Equation 27  *** sign changed from prev implementation???

	// error_roll_pitch are in Accel m/s^2 units
	// Limit max error_roll_pitch to limit max omega_P and omega_I
	_error_roll_pitch.x = dcm_constrain(_error_roll_pitch.x, -1.17f, 1.17f);
	_error_roll_pitch.y = dcm_constrain(_error_roll_pitch.y, -1.17f, 1.17f);
	_error_roll_pitch.z = dcm_constrain(_error_roll_pitch.z, -1.17f, 1.17f);

	_omega_P = _error_roll_pitch * (Kp_ROLLPITCH * accel_weight);
	_omega_I += _error_roll_pitch * (Ki_ROLLPITCH * accel_weight);

	_error_yaw = _dcm_matrix.c * error_course;	// Equation 24, Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	_omega_P += _error_yaw * Kp_YAW;			// Adding yaw correction to proportional correction vector.
	_omega_I += _error_yaw * Ki_YAW;			// adding yaw correction to integrator correction vector.

	//	Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
	integrator_magnitude = _omega_I.length();
	if (integrator_magnitude > radians(300)) {
		_omega_I *= (0.5f * radians(300) / integrator_magnitude);		// Why do we have this discontinuous?  EG, why the 0.5?
	}
	//Serial.print("*");
}


/**************************************************/
void
AP_DCM::euler_angles(void)
{
	pitch 		= -asin(_dcm_matrix.c.x);
	roll 		= atan2(_dcm_matrix.c.y, _dcm_matrix.c.z);
	yaw 		= atan2(_dcm_matrix.b.x, _dcm_matrix.a.x);

	float r, p, y;
	_dcm_matrix.to_euler(&r,&p,&y);

	roll=r;
	pitch=p;
	yaw=y;

	roll_sensor 	= degrees(roll)  * 100;
	pitch_sensor 	= degrees(pitch) * 100;
	yaw_sensor 		= degrees(yaw)   * 100;

	if (yaw_sensor < 0)
		yaw_sensor += 36000;
}

float
AP_DCM::get_health(void)
{
	return _health;
}

// degrees -> radians
float
AP_DCM::radians(float deg)
{
    return deg * DEG_TO_RAD;
}

// radians -> degrees
float
AP_DCM::degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

float
AP_DCM::dcm_constrain(float m,float a,float b)
{
	if(m<=a)        m=a;
	else if(m>=b)   m=b;

	return m;
}

