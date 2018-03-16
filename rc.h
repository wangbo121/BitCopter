/*
 * rc.h
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#ifndef RC_H_
#define RC_H_

#include <stdint.h>

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

class AP_RC
{
  private:
  public:
	AP_RC();
	void 		init();
	void 		output_ch_pwm(uint8_t ch, uint16_t pwm);
	uint16_t 	input_ch(uint8_t ch);
};



int rc_init();

/*
 * rc是读取遥控信号的抽象概念
 * rc可以从futaba，天地飞等遥控器读取，也可以从地面站读取遥控信号
 */
int rc_read_data();



#endif /* RC_H_ */
