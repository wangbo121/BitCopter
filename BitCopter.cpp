/*
 * BitCopter.cpp
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#include "Copter.h"

/*
 * 这是任务调度表，除了fast_loop中的任务，其他任务都在这里执行
 * 中间的数字是执行频率，也就是经过多少个tick执行一次这个任务(目前我写的是10ms一个tick)
 * 最右边的数字是最大允许的时间，单位是微妙
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
  1 = 100hz
  2 = 50hz
  4 = 25hz
  10 = 10hz
  100 = 1hz
 */
#define SCHED_TASK(func) (void (*)())&Copter::func

const BIT_Scheduler::Task Copter::scheduler_tasks[] =
{
//      { SCHED_TASK(update_GPS),                                                  10,     900 },
//      { SCHED_TASK(set_rc_out),                                                    100,     100 },
//
//      { SCHED_TASK(send_ap2gcs_cmd_boatlink),                          1,    1000 },
//      { SCHED_TASK(send_ap2gcs_wp_boatlink),                            1,    1000 },
//      { SCHED_TASK(send_ap2gcs_realtime_data_boatlink),    100,    1000 },
//      { SCHED_TASK(send_ap2gcs_realtime_data_boatlink_by_udp),    100,    1000 },
//
//      { SCHED_TASK(record_log),                                                   100,    1100 },
//      { SCHED_TASK(record_wp),                                                   100,    1100 },
//      { SCHED_TASK(record_config),                                                   100,    1100 },
//      { SCHED_TASK(get_timedata_now),                                     100,    1100 },
      { SCHED_TASK(loop_slow),                                                    1000,    1100 },

      { SCHED_TASK(end_of_task),                                               1000,    1100 }
};

#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms，对应每个循环100hz
int seconds=0;
int micro_seconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick的微秒数*/
struct timeval maintask_tick;

int main(int argc,char * const argv[])
{
    printf("Welcome to BitPilot \n");

    /*
     * 1外围的硬件设备初始化，这些硬件是底层设备，跟飞控软件分开
     * 2飞控程序的初始化，初始化导航制导控制参数等以及全局参数
     */
    copter.setup();

    /*
     * 除了姿态控制环之外的其他任务的调度表
     */
    copter.scheduler.init(&copter.scheduler_tasks[0], sizeof(copter.scheduler_tasks)/sizeof(copter.scheduler_tasks[0]));
    printf("There are %ld task to run!!!\n",sizeof(copter.scheduler_tasks)/sizeof(copter.scheduler_tasks[0]));

    while (1)
    {
    	/*
    	 * select定时器，每10ms运行一次loop
    	 */
        maintask_tick.tv_sec = seconds;
        maintask_tick.tv_usec = micro_seconds;
        select(0, NULL, NULL, NULL, &maintask_tick);

        copter.loop();
    }

    return 0;
}

void Copter::loop( void )
{
	static uint8_t loop_cnt;

	loop_cnt++;

    uint32_t timer = (uint32_t)gettimeofday_us();//当前系统运行时间精确到微秒

    loop_fast();//在无人机中是姿态控制内环，在无人船中是制导控制环

    //  告诉调度器scheduler一个tick已经过去了，目前1个tick指的是10毫秒
    scheduler.tick();

    /*
     * loop_us这个应该是一个tick循环所指定的时间，比如我这里目前定义的是10ms，
     * 但是如果所有任务的执行时间的总和还是小于10ms呢，如果没有select这个定时器，就会一直循环
     * 这样就会导致scheduler_tasks数组中指定的频率失去原本的意义，所以必须有select定时器或者
     * 如果在单片机中，则使用某一个定时器来触发这个loop这个函数
     */
    uint32_t loop_us = micro_seconds;
    uint32_t time_available = loop_us - ( (uint32_t)gettimeofday_us() - timer );

    if(loop_cnt > 100)
	{
		loop_cnt = 0;
	}

    scheduler.run(time_available > loop_us ? 0u : time_available);
}

void Copter::loop_fast()
{
	/*
	 * 将来跟硬件驱动获取数据整合时,这个函数是不需要的,现在是模拟,所以才需要
	 * 20170918添加了all_external_device_input和output一直循环从驱动中获取数据，
	 * 至于硬件驱动到底多大频率获取的我不管，我只是每次从这里获取数据
	 */
	update_all_external_device_input();

	/*
	 * wangbo20170801
	 * 其实如果只是增稳控制的话
	 * 只需要下面5步骤就可以了
	 * 其他的都是用来与地面站通信然后实现自动驾驶的，比如气压计，空速计，gps，导航，航点等
	 * 1--read_radio
	 * 2--update_DCM
	 * 3--update_current_flight_mode
	 * 4--control根据飞行模式 control_mode的选项，选择不同的控制方式
	 * 5--set_servos
	 */

	G_Dt=0.01;//G_Dt是dcm积分要用的，这个设置为0.01秒也就是100hz主要是为了跟sim_aircraft的速率一致，但是其实20ms(50hz)就够

	/*
	 * 1--读取接收机的信号，获取遥控器各个通道
	 */
	read_radio();
	//下面的设置遥控器其实是不需要的，应该按照从地面站或者从遥控器的第五通道来决定飞行模式
	//g.channel_rudder.set_pwm(1600);//这个set_pwm参数的范围是1000～2000
	//g.channel_pitch.set_pwm(1600);//这个set_pwm参数的范围是1000～2000，把pitch一直设置为1600，看能不能稳定在9度左右
	//g.rc_5.set_pwm(1400);//rc_5大于1500时，是增稳控制状态
	//g.rc_5.set_pwm(1600);//rc_5大于1500时，是增稳控制状态
	//g.rc_5.set_pwm(1990);//rc_5大于1900时，是绕航点飞行状态

	/*
	 * 3--刷新控制状态，从而选择控制方式
	 * 设置yaw_mode roll_pitch_mode throttle_mode的模式
	 * 然后update_roll_pitch_mode，update_yaw_mode，update_throttle_mode要用
	 */
	update_current_flight_mode();

	/*
	 * 4--把期望的roll pitch yaw作用于飞机
	 */
	switch(control_mode)
	{
	case STABILIZE:
		/*
		* 先是roll pitch yaw的2级pid控制
		* 再是油门throttle的2级pid控制
		* 都是只是计算得出g.channel.servo_out的值
		* 在motors_output时再把这些计算的值真正输出
		* update_roll_pitch_mode和update_yaw_mode都是只有p控制器，计算得到目标姿态角度
		*/
		update_roll_pitch_mode();
		update_yaw_mode();//上面这两个函数有问题呀，上面两个函数赋值给的是EARTH_FRAME，但是下面的run_rate_controllers是用的BODY_FRAME，所以还需要仔细再看一下apm

		//这个是更新内环的速率控制器的目标，update targets to rate controllers
		update_rate_contoller_targets();//这个步骤很重要，是把上面的earth坐标系下的计算数值量转为机体坐标系下的

		//这个是执行了角速度的控制器，需要从ahrs或者imu获取角速度的大小，扩大了100倍，这个函数还得看一下
		run_rate_controllers();

		//这个是油门的控制，跟姿态的控制分开，油门的更新速率不需要那么快，油门的更新放在了medium_loop中了，5分之1的loop的频率，如果是50hz的话，那么就是10hz，100ms更新一次
		update_throttle_mode();//计算油门量的输出值，这个先放在这里，按道理应该放在50hz的循环里
		break;
	case ACRO:
		// call rate controllers
		g.channel_roll.servo_out = g.channel_roll.control_in;
		g.channel_pitch.servo_out = g.channel_pitch.control_in;
		g.channel_rudder.servo_out = g.channel_rudder.control_in;

		g.channel_throttle.servo_out=g.channel_throttle.control_in;
		break;
	default:
		break;
	}

	/* 5--把计算所得控制量输出给电机 */
	motors_output();

	/*
	 * 2--更新姿态，获取飞机现在的姿态角
	 */
	ahrs.update_DCM(G_Dt);

	if(takeoff_complete == false)
	{
		//没有起飞之前，把所有的积分项都清零
		// reset these I terms to prevent awkward tipping on takeoff
		reset_rate_I();
		reset_stability_I();
	}
}
