/*
 * read_futaba.h
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#ifndef READ_FUTABA_H_
#define READ_FUTABA_H_

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

int init_futaba();
int read_futaba_data(unsigned char *buf, unsigned int len);



#endif /* READ_FUTABA_H_ */
