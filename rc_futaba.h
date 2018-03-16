/*
 * rc_futaba.h
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#ifndef RC_FUTABA_H_
#define RC_FUTABA_H_

int futaba_init();

/*
 * 遥控信号属于外围设备
 * 该函数读取futaba遥控器的信号，
 * 并将遥控器信号存入到all_external_device_input中去
 */
int futaba_read_data();



#endif /* RC_FUTABA_H_ */
