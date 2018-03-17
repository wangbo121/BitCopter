/*
 * ahrs_DCM.h
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#ifndef AHRS_DCM_H_
#define AHRS_DCM_H_

#include <inttypes.h>

#include "BIT_MATH.h"
#include "IMU.h"

#ifndef  M_PI
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
#endif

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

class AP_DCM
{
public:
#if 0
	// Constructors
	AP_DCM(IMU &imu ) :
		_imu(imu),
		_dcm_matrix(1, 0, 0,
					0, 1, 0,
					0, 0, 1),
		_course_over_ground_x(0),
		_course_over_ground_y(1)
	{}
#else
	// Constructors
	AP_DCM() :
		_dcm_matrix(1, 0, 0,
					0, 1, 0,
					0, 0, 1),
		_course_over_ground_x(0),
		_course_over_ground_y(1)
	{}
#endif
	//AP_DCM();


	// Accessors
	Vector3f	get_gyro(void) {return _omega_integ_corr; }		// We return the raw gyro vector corrected for bias
	Vector3f	get_accel(void) { return _accel_vector; }
	Matrix3f	get_dcm_matrix(void) {return _dcm_matrix; }
	Matrix3f	get_dcm_transposed(void) {Matrix3f temp = _dcm_matrix;  return temp.transposed();}

	void		set_centripetal(bool b) {_centripetal = b;}
	bool		get_centripetal(void) {return _centripetal;}

	float radians(float deg);
	float degrees(float rad);
	float dcm_constrain(float m,float a,float b);


	// Methods
	void 		update_DCM(float _G_Dt);

	float		get_health(void);

	int32_t		roll_sensor;					// Degrees * 100
	int32_t		pitch_sensor;					// Degrees * 100
	int32_t		yaw_sensor;						// Degrees * 100

	float		roll;							// Radians
	float		pitch;							// Radians
	float		yaw;							// Radians

	uint8_t 	gyro_sat_count;
	uint8_t 	renorm_sqrt_count;
	uint8_t 	renorm_blowup_count;

private:
	// Methods
	void 		read_adc_raw(void);
	void 		accel_adjust(void);
	float 		read_adc(int select);
	void 		matrix_update(float _G_Dt);
	void 		normalize(void);
	Vector3f 	renorm(Vector3f const &a, int &problem);
	void 		drift_correction(void);
	void 		euler_angles(void);

	IMU 		_imu;//这个必须第一位，因为定义ahrs_DCM时用到了初始化

	Matrix3f	_dcm_matrix;

	Vector3f 	_accel_vector;				// Store the acceleration in a vector
	Vector3f 	_gyro_vector;				// Store the gyros turn rate in a vector
	Vector3f	_omega_P;					// Omega Proportional correction
	Vector3f 	_omega_I;					// Omega Integrator correction
	Vector3f 	_omega_integ_corr;			// Partially corrected Gyro_Vector data - used for centrepetal correction
	Vector3f 	_omega;						// Corrected Gyro_Vector data
	Vector3f 	_error_roll_pitch;
	Vector3f 	_error_yaw;
	float 		_errorCourse;
	float 		_course_over_ground_x; 		// Course overground X axis
	float 		_course_over_ground_y; 		// Course overground Y axis
	float		_health;
	bool		_centripetal;
};



#endif /* AHRS_DCM_H_ */
