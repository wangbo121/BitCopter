/*
 * IMU_6050.h
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#ifndef IMU_6050_H_
#define IMU_6050_H_

/*
 * IMU_6050属于外围设备
 * IMU_6050是具体的惯性测量单元设备
 */
int IMU_6050_init();

void IMU_6050_read_data();


#endif /* IMU_6050_H_ */
