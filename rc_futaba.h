/*
 * rc_futaba.h
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#ifndef RC_FUTABA_H_
#define RC_FUTABA_H_

//int futaba_init();
//
//
//int futaba_read_data();

#define UART_FUTABA_BAUD       100000
#define UART_FUTABA_DATABITS 8 //8 data bit
#define UART_FUTABA_STOPBITS 2 //1 stop bit
#define UART_FUTABA_PARITY     'E' //no parity

struct T_FUTABA
{
	unsigned short year;
	unsigned char month;
	unsigned char date;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
};

extern struct T_FUTABA futaba;

int rc_futaba_init();

/*
 * 遥控信号属于外围设备
 * 该函数读取futaba遥控器的信号，
 * 并将遥控器信号存入到all_external_device_input中去
 */
int rc_futaba_read_data(unsigned char *buf, unsigned int len);




#endif /* RC_FUTABA_H_ */
