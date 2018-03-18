/*
 * IMU_6050.cpp
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#include "uart.h"

#include "IMU_6050.h"

static struct T_UART_DEVICE uart_device_mpu_6050;

#define UART_MPU_6050 "/dev/ttyUSB0"

#define UART_MPU_6050_BAUD       9600
#define UART_MPU_6050_DATABITS 8 //8 data bit
#define UART_MPU_6050_STOPBITS 1 //1 stop bit
#define UART_MPU_6050_PARITY     'N' //no parity

#define UART_BUF_MPU_6050_SIZE 512

int IMU_6050_process_data(unsigned  char *buf, unsigned int len);

int IMU_6050_init()
{
	/*
	 * 打开mpu_6050对应的串口
	 */
	uart_device_mpu_6050.uart_name=UART_MPU_6050;

	uart_device_mpu_6050.baudrate=UART_MPU_6050_BAUD;
	uart_device_mpu_6050.databits=UART_MPU_6050_DATABITS;
	uart_device_mpu_6050.parity=UART_MPU_6050_PARITY;
	uart_device_mpu_6050.stopbits=UART_MPU_6050_STOPBITS;

	/*
	 * 20180205因为我的电脑就1个串口还得用，所以暂时把这个打开串口和创建串口线程注释掉了
	 */
	uart_device_mpu_6050.uart_num=open_uart_dev(uart_device_mpu_6050.uart_name);

	uart_device_mpu_6050.ptr_fun=IMU_6050_process_data;

	set_uart_opt( uart_device_mpu_6050.uart_name, \
						  uart_device_mpu_6050.baudrate,\
						  uart_device_mpu_6050.databits,\
						  uart_device_mpu_6050.parity,\
						  uart_device_mpu_6050.stopbits);

	//create_uart_pthread(&uart_device_futaba);

	return 0;
}

#define IMU_6050_RECV_HEAD1  0
#define IMU_6050_RECV_HEAD2  1


#define RADIO_RECV_DATA 8
#define RADIO_RECV_CHECKSUM 9
#define RADIO_RECV_CHECKSUM1 10
#define RADIO_RECV_CHECKSUM2 11
#define RADIO_RECV_CHECKSUM 12

static int radio_recv_state = 0;

int IMU_6050_process_data( unsigned char *buf, unsigned int len)
{

}

void IMU_6050_read_data()
{
	/*
	 * 解析mpu6050的串口数据
	 * 得到线加速度和角速度
	 * 把解析后的数据存放到all_external_device_input中
	 */
	char buf[UART_BUF_MPU_6050_SIZE] = { 0 };
	int read_len;

	struct T_UART_DEVICE *ptr_uart;

	ptr_uart = &uart_device_mpu_6050;

	//if(-1!=(read_len=read_uart_data(ptr_uart->uart_name, buf, 200, sizeof(buf)-1)))
	if(-1!=(read_len=read_uart_data(ptr_uart->uart_name, buf, UART_BUF_MPU_6050_SIZE, sizeof(buf)-1)))
	{
		#if 0
		printf("read_len=%d\n",read_len);
		buf[read_len]='\0';
		printf("%s\n",buf);
		#endif
		if(read_len>0)
		{
			ptr_uart->ptr_fun((unsigned char *)buf, (unsigned int)read_len);
		}
	}
}



