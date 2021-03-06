/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_loop.c
 * 描述    ：任务调度函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "loop.h"
#include "ANO_Drv_MPU6050.h"
#include "ANO_IMU.h"
#include "ANO_Data_Transfer.h"
#include "ANO_RC.h"
#include "ANO_CTRL.h"
#include "ANO_Voltage.h"
#include "receive.h"
#include "nrf24l01.h"
loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
    loop.time++; //u16
    loop.cnt_2ms++;
    loop.cnt_6ms++;
    loop.cnt_10ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;

    if( loop.check_flag >= 2)
    {
        loop.err_flag ++;     //每累加一次，证明代码在预定周期2ms内没有跑完。
    }
    else
    {
        loop.check_flag += 1;	//该标志位在循环的最后被清零
    }
    LED_1ms_DRV(); 				//1ms中断里边，1ms执行一次
}

u32 test_time[10];
void Duty_2ms()
{
    ANO_DT_Data_Exchange(); 			//数据交换

    test_time[0] = GetSysTime_us();

    MPU6050_Data_Prepare(0.002f); //6050传感器数据准备


    RC_duty(0.002f); 							//遥控数据处理
//		g_v_ImuUpdate(&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data);
    /*=====================================================================================================================
    						CH_N  1横滚，2俯仰，3油门，4航向 范围：+-500
    =====================================================================================================================*/
    CTRL_Duty((float)CH_N[ROL],(float)CH_N[PIT],(float)CH_N[THR],(float)CH_N[YAW]);//2ms

    test_time[1] = GetSysTime_us();
    test_time[2] = test_time[1] - test_time[0];
}


void Duty_6ms()
{
    test_time[3] = GetSysTime_us();
    IMU_update(0.006f,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data); //姿态解算
    test_time[4] = GetSysTime_us();
    test_time[5] = test_time[4] - test_time[3];


}
void Duty_10ms()
{

}

void Duty_20ms()
{
    LED_Dyty(0.02f);
}

void Duty_50ms()
{
    PID_Save_Overtime(1500,50);

    voltage_check();
}


void main_loop()
{
    if( loop.check_flag >= 1 )
    {

        //Duty_1ms();							//周期1ms的任务

        if( loop.cnt_2ms >= 2 )
        {
            loop.cnt_2ms = 0;
            Duty_2ms();						//周期2ms的任务
        }
        if( loop.cnt_6ms >= 6 )
        {
            loop.cnt_6ms = 0;
            Duty_6ms();						//周期5ms的任务
        }
        if( loop.cnt_10ms >= 10 )
        {
            loop.cnt_10ms = 0;
            Duty_10ms();					//周期10ms的任务
        }
        if( loop.cnt_20ms >= 20 )
        {
            loop.cnt_20ms = 0;
            Duty_20ms();					//周期20ms的任务
        }
        if( loop.cnt_50ms >= 50 )
        {
            loop.cnt_50ms = 0;
            Duty_50ms();					//周期50ms的任务
        }

        loop.check_flag = 0;		//循环运行完毕标志
    }
    Nrf_Check_Event();
//    Usb_Hid_Receive();//主循环里不断查询


}
/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/


