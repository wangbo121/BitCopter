/*
 * read_futaba.cpp
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>

#include "uart.h"
#include "global.h"

#include "read_futaba.h"

static struct T_UART_DEVICE uart_device_futaba;

int init_futaba()
{
	uart_device_futaba.uart_name=UART_FUTABA;

	uart_device_futaba.baudrate=UART_FUTABA_BAUD;
	uart_device_futaba.databits=UART_FUTABA_DATABITS;
	uart_device_futaba.parity=UART_FUTABA_PARITY;
	uart_device_futaba.stopbits=UART_FUTABA_STOPBITS;

	/*
	 * 20180205因为我的电脑就1个串口还得用，所以暂时把这个打开串口和创建串口线程注释掉了
	 */
    uart_device_futaba.uart_num=open_uart_dev(uart_device_futaba.uart_name);

    uart_device_futaba.ptr_fun=read_futaba_data;

    set_uart_opt( uart_device_futaba.uart_name, \
                  uart_device_futaba.baudrate,\
                  uart_device_futaba.databits,\
                  uart_device_futaba.parity,\
                  uart_device_futaba.stopbits);

    create_uart_pthread(&uart_device_futaba);

    return 0;
}

int read_futaba_data(unsigned char *buf, unsigned int len)
{

#if 1
	char buf_temp[200];
	memcpy(buf_temp, buf, len);
	buf_temp[len+1]='\0';

	printf("futaba收到的数据：\n");

	for (unsigned int i = 0; i < len; i++)
	{
		printf("  %x", buf_temp[i]);
	}
	printf("\n");
#endif

	return 0;
}


