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




#endif /* GLOBAL_H_ */
