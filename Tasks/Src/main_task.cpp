/*
 * @Author: rogue-wave zhangjingjie@zju.edu.cn
 * @Date: 2025-11-22 21:11:08
 * @LastEditors: rogue-wave zhangjingjie@zju.edu.cn
 * @LastEditTime: 2025-11-29 17:38:16
 * @FilePath: \dipan_quanxiang\Tasks\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: rogue-wave zhangjingjie@zju.edu.cn
 * @Date: 2025-11-22 21:11:08
 * @LastEditors: rogue-wave zhangjingjie@zju.edu.cn
 * @LastEditTime: 2025-11-26 23:01:04
 * @FilePath: \dipan_quanxiang\Tasks\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE#in
 */
/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team,Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "pid.hpp"
#include "duo.hpp"
#include "dipan.hpp"
#include "yaw.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*                    四个轮电机的速度环                   */
pid::Pid pid_lun_id_1(25,0,0,16000,-16000);
pid::Pid pid_lun_id_2(20,0,0,16000,-16000);
pid::Pid pid_lun_id_3(20,0,0,16000,-16000);
pid::Pid pid_lun_id_4(10,0,0,16000,-16000);
//                    四个舵机的角度环                   */
pid::Pid pid_duo_angle_id_1(100.0f,0.0f,1.0f,120.0f,-120.0f);
pid::Pid pid_duo_angle_id_2(110.0f,0.0f,1.0f,120.0f,-120.0f);
pid::Pid pid_duo_angle_id_3(100.0f,0.0f,1.0f,120.0f,-120.0f);
pid::Pid pid_duo_angle_id_4(100.0f,0.0f,1.0f,120.0f,-120.0f);
//                    四个舵机的速度环                   */
pid::Pid pid_duo_vel_id_1(105.0f,1.0f,11.0f,15000,-15000);
pid::Pid pid_duo_vel_id_2(105.0f,1.0f,11.0f,15000,-15000);
pid::Pid pid_duo_vel_id_3(105.0f,1.0f,11.0f,15000,-15000);
pid::Pid pid_duo_vel_id_4(105.0f,1.0f,11.0f,15000,-15000);
//                    云台的角度环和速度环                   */
pid::Pid pid_yaw_vel(1.5f,0.0f,0.0f,7.0f,-7.0f);
pid::Pid pid_yaw_angle(10.0f,0.0f,0.0f,15.0f,-15.0f);
//                    四个舵电机的初始化                */
duo::Duo duo_id_1(649.0f);
duo::Duo duo_id_2(6803.0f);
duo::Duo duo_id_3(4090.0f);
duo::Duo duo_id_4(2041.0f);
//                   跟随模式的位置环                  */
pid::Pid pid_dipan_follow_pos(4.0f,0.0f,0.0f,10.0f,-10.0f);
//                  底盘初始化                 */
dipan::Dipan dipan_private;
//                  云台yaw电机初始化                 */
Joint_Motor_t yaw_private;
//                  云台yaw电机数据处理变量初始化                 */
yaw::Yaw yaw_angle_data;
/* External variables --------------------------------------------------------*/
/*                  四个轮电机的当前速度                                       */
extern float rpm;
extern float rpm_2;
extern float rpm_3;
extern float rpm_4;
/*                  IMU（云台）的当前旋转速度与角度                                       */
extern float vel_yaw;
extern float imu_yaw;
/*                  四个舵电机的当前角度与速度                                       */
extern float rpm_duo_1;
extern float rpm_duo_2;
extern float rpm_duo_3;
extern float rpm_duo_4;
extern float angle_duo_1;
extern float angle_duo_2;
extern float angle_duo_3;
extern float angle_duo_4;
/*                  遥控器数据                                       */

extern float rc_lv_private;
extern float rc_lh_private;
extern float rc_rh_private;
extern float rc_rv_private;
//                  dm电机返回值                                       */
extern float angle_yaw;
/*              发送数据                 */
uint8_t temp_lun[8]={0,0,0,0,0,0,0,0};
uint8_t temp_duo[8]={0,0,0,0,0,0,0,0};
/* Private function prototypes -----------------------------------------------*/
void ModeIwdg(void);
uint32_t tick = 0;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr;

void RobotInit(void) { rc_ptr = new remote_control::DT7(); }

void MainInit(void) {
  RobotInit();

  // 开启CAN1和CAN2
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
}
float a = 0.0f;
float duo_purpose_angle = 0.0f;
float duo_now_id_1_angle_output = 0;
float duo_now_id_2_angle_output = 0;
float duo_now_id_3_angle_output = 0;
float duo_now_id_4_angle_output = 0;
int f_yaw = 0;
float vel_error_purpose = 0.0f;
void MainTask(void) {
  tick++;
  if(tick<1000)
  {
    ModeIwdg();
    return;
  }  
/*              电机初始化                 */
  if (f_yaw == 0)
  {
    enable_motor_mode(&hcan2,0x01,MIT_MODE);
    f_yaw = 1;
  }

  /*                 舵电机pid计算与error计算                      */
  yaw_angle_data.set_imu_now_yaw(imu_yaw);
  yaw_angle_data.set_purpose_yaw(-rc_rh_private);
  float yaw_angle_error_temp = yaw_angle_data.calc_yaw_error();
  pid_yaw_angle.set_error(yaw_angle_error_temp);
  vel_error_purpose = pid_yaw_angle.calc();
  pid_yaw_vel.set_error(vel_error_purpose-vel_yaw);
  float yaw_temp_vel_output = pid_yaw_vel.calc();
  mit_ctrl(&hcan2,0x01,0,0,0,0,yaw_temp_vel_output);


/*                    遥控器防抖                        */
  if(abs(rc_lh_private)<0.05)
  {
    rc_lh_private = 0.0f;
  }
  if(abs(rc_lv_private)<0.05)
  {
    rc_lv_private = 0.0f;
  }
/*                     跟随模式的位置环计算                        */
  if(-angle_yaw+1.52f >= 3.14f)
  {
    pid_dipan_follow_pos.set_error(-angle_yaw+1.52f-6.28f);
  }
  else
  {
    pid_dipan_follow_pos.set_error(-angle_yaw+1.52f);
  }
  float dipan_w = pid_dipan_follow_pos.calc();

/*             底盘解算                */
//note:v_temp是返回值速度，angle_temp是返回值角度
  dipan_private.set_vy(rc_lv_private*1900.0f);
  dipan_private.set_vx(rc_lh_private*1900.0f);
  dipan_private.set_w(dipan_w);
  dipan_private.calc_jiesuan();
  float v_temp[4] = {0.0f,0.0f,0.0f,0.0f};
  float angle_temp[4] = {0.0f,0.0f,0.0f,0.0f};
  dipan_private.get_v(v_temp);
  dipan_private.get_angle(angle_temp);



  /*              四个舵电机的pid计算                 */
  //Note:第一：为了相应迅速，这里会去选取最近的方位，要是速度相反直接使得速度反向
  duo_id_1.set_now_angle(angle_duo_1);
  duo_id_1.set_purpose_angle(angle_temp[0]);
  duo_id_1.calc_error();
  float output_temp_duo = duo_id_1.get_error();
  if(abs(output_temp_duo)>3.14/2.0f)
  {
    if(output_temp_duo<0.0f)
    {
      pid_duo_angle_id_1.set_error(duo_id_1.get_error()+3.14f);
      v_temp[0] = -v_temp[0];
      v_temp[0] = v_temp[0]*cos((duo_id_1.get_error()+3.14f)/3.0f);
    }
    else
    {
      pid_duo_angle_id_1.set_error(duo_id_1.get_error()-3.14f);
      v_temp[0] = -v_temp[0];
      v_temp[0] = v_temp[0]*cos((duo_id_1.get_error()-3.14f)/3.0f);
    }
  }
  else
  {
    pid_duo_angle_id_1.set_error(duo_id_1.get_error());
    v_temp[0] = v_temp[0]*cos((duo_id_1.get_error())/3.0f);
  }
  duo_now_id_1_angle_output = pid_duo_angle_id_1.calc();
  


  duo_id_2.set_now_angle(angle_duo_2);
  duo_id_2.set_purpose_angle(angle_temp[1]);
  duo_id_2.calc_error();
  output_temp_duo = duo_id_2.get_error();
  if(abs(output_temp_duo)>3.14/2.0f)
  {
    if(output_temp_duo<0.0f)
    {
      pid_duo_angle_id_2.set_error(duo_id_2.get_error()+3.14f);
      v_temp[1] = -v_temp[1];
      v_temp[1] = v_temp[1]*cos((duo_id_2.get_error()+3.14f)/3.0f);
    }
    else
    {
      pid_duo_angle_id_2.set_error(duo_id_2.get_error()-3.14f);
      v_temp[1] = -v_temp[1];
      v_temp[1] = v_temp[1]*cos((duo_id_2.get_error()-3.14f)/3.0f);
    }
  }
  else
  {
    pid_duo_angle_id_2.set_error(duo_id_2.get_error());
    v_temp[1] = v_temp[1]*cos((duo_id_2.get_error())/3.0f);
  }
  duo_now_id_2_angle_output = pid_duo_angle_id_2.calc();
  

  duo_id_3.set_now_angle(angle_duo_3);
  duo_id_3.set_purpose_angle(angle_temp[2]);
  duo_id_3.calc_error();
  output_temp_duo = duo_id_3.get_error();
  if(abs(output_temp_duo)>3.14/2.0f)
  {
    if(output_temp_duo<0.0f)
    {
      pid_duo_angle_id_3.set_error(duo_id_3.get_error()+3.14f);
      v_temp[2] = -v_temp[2];
      v_temp[2] = v_temp[2]*cos((duo_id_3.get_error()+3.14f)/3.0f);
    }
    else
    {
      pid_duo_angle_id_3.set_error(duo_id_3.get_error()-3.14f);
      v_temp[2] = -v_temp[2];
      v_temp[2] = v_temp[2]*cos((duo_id_3.get_error()-3.14f)/3.0f);
    }
  }
  else
  {
    pid_duo_angle_id_3.set_error(duo_id_3.get_error());
    v_temp[2] = v_temp[2]*cos((duo_id_3.get_error())/3.0f);
  }
  duo_now_id_3_angle_output = pid_duo_angle_id_3.calc();

  duo_id_4.set_now_angle(angle_duo_4);
  duo_id_4.set_purpose_angle(angle_temp[3]);
  duo_id_4.calc_error();
  output_temp_duo = duo_id_4.get_error();
  if(abs(output_temp_duo)>3.14/2.0f)
  {
    if(output_temp_duo<0.0f)
    {
      pid_duo_angle_id_4.set_error(duo_id_4.get_error()+3.14f);
      v_temp[3] = -v_temp[3];
      v_temp[3] = v_temp[3]*cos((duo_id_4.get_error()+3.14f)/3.0f);
    }
    else
    {
      pid_duo_angle_id_4.set_error(duo_id_4.get_error()-3.14f);
      v_temp[3] = -v_temp[3];
      v_temp[3] = v_temp[3]*cos((duo_id_4.get_error()-3.14f)/3.0f);
    }
  }
  else
  {
    pid_duo_angle_id_4.set_error(duo_id_4.get_error());
    v_temp[3] = v_temp[3]*cos((duo_id_4.get_error())/3.0f);
  }
  duo_now_id_4_angle_output = pid_duo_angle_id_4.calc();
/*             轮子设置力矩                */
//note:为了降低漂移，有一个角度误差限制，只有角度差够小才会转
//注意一下，差值不能太小，不然无法转动
  if(abs(angle_duo_1-angle_temp[0])<0.001f)
  {
    v_temp[0] = 0.0f;
  }
  pid_lun_id_1.set_error(v_temp[0]-rpm);
  float output = pid_lun_id_1.calc();
  int16_t b = (int16_t)output;
  temp_lun[0]=(uint8_t)(b>>8);
  temp_lun[1]=(uint8_t)(b);
  if(abs(angle_duo_2-angle_temp[1])<0.001f)
  {
    v_temp[1] = 0.0f;
  }
  pid_lun_id_2.set_error(v_temp[1]-rpm_2);
  output = pid_lun_id_2.calc();
  b = (int16_t)output;
  temp_lun[2]=(uint8_t)(b>>8);
  temp_lun[3]=(uint8_t)(b);
  if(abs(angle_duo_3-angle_temp[2])<0.001f)
  {
    v_temp[2] = 0.0f;
  }
  pid_lun_id_3.set_error(v_temp[2]-rpm_3);
  output = pid_lun_id_3.calc();
  b = (int16_t)output;
  temp_lun[4]=(uint8_t)(b>>8);
  temp_lun[5]=(uint8_t)(b);
  if(abs(angle_duo_4-angle_temp[3])<0.001f)
  {
    v_temp[3] = 0.0f;
  }
  pid_lun_id_4.set_error(v_temp[3]-rpm_4);
  output = pid_lun_id_4.calc();
  b = (int16_t)output;
  temp_lun[6]=(uint8_t)(b>>8);
  temp_lun[7]=(uint8_t)(b);
/*             舵电机设置力矩                */
  pid_duo_vel_id_1.set_error(duo_now_id_1_angle_output - rpm_duo_1);
  output = pid_duo_vel_id_1.calc();
  b = (int16_t)output;
  temp_duo[0]=(uint8_t)(b>>8);
  temp_duo[1]=(uint8_t)(b);

  pid_duo_vel_id_2.set_error(duo_now_id_2_angle_output - rpm_duo_2);
  output = pid_duo_vel_id_2.calc();
  b = (int16_t)output;
  temp_duo[2]=(uint8_t)(b>>8);
  temp_duo[3]=(uint8_t)(b);

  pid_duo_vel_id_3.set_error(duo_now_id_3_angle_output - rpm_duo_3);
  output = pid_duo_vel_id_3.calc();
  b = (int16_t)output;
  temp_duo[4]=(uint8_t)(b>>8);
  temp_duo[5]=(uint8_t)(b);

  pid_duo_vel_id_4.set_error(duo_now_id_4_angle_output - rpm_duo_4);
  output = pid_duo_vel_id_4.calc();
  b = (int16_t)output;
  temp_duo[6]=(uint8_t)(b>>8);
  temp_duo[7]=(uint8_t)(b);
  
  CAN_Send_Msg(&hcan2,temp_lun,0x200,8);
  CAN_Send_Msg(&hcan1,temp_duo,0x1FE,8);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim6) {
    MainTask();
  }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart3) {
    if (Size == remote_control::kRcRxDataLen) {
      // TODO:在这里刷新看门狗
      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
  }
}
/**
 * @brief 独立看门狗模式函数
 * 该函数用于配置和启动独立看门狗模式
 * 通过CAN总线发送特定消息来激活看门狗功能
 */
void ModeIwdg(void) {
  uint8_t kong[8]={0,0,0,0,0,0,0,0};
  mit_ctrl(&hcan2,0x01,0,0,0,0,0);  // 定义一个8字节的零数组，用于发送空消息
  CAN_Send_Msg(&hcan2,kong,0x200,8);   // 通过CAN2总线发送ID为0x200的8字节空消息
  CAN_Send_Msg(&hcan1,kong,0x1FE,8);   // 通过CAN1总线发送ID为0x1FE的8字节空消息
}
