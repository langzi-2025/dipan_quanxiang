/*
 * @Author: rogue-wave zhangjingjie@zju.edu.cn
 * @Date: 2025-11-22 21:11:08
 * @LastEditors: rogue-wave zhangjingjie@zju.edu.cn
 * @LastEditTime: 2025-11-24 21:55:28
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
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
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
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pid::Pid pid_lun_id_1(25,0,0,16000,-16000);
pid::Pid pid_lun_id_2(20,0,0,16000,-16000);
pid::Pid pid_lun_id_3(20,0,0,16000,-16000);
pid::Pid pid_lun_id_4(20,0,0,16000,-16000);
/* External variables --------------------------------------------------------*/
extern float rpm;
extern float rpm_2;
extern float rpm_3;
extern float rpm_4;
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
void MainTask(void) {
   tick++;
  if(tick<1000)
  {
    ModeIwdg();
    return;
  }  
  uint8_t kong[8]={0,0,0,0,0,0,0,0};
  uint8_t temp[8]={0,0,0,0,0,0,0,0};
  
  pid_lun_id_1.ser_error(0.0f-rpm);
  float output = pid_lun_id_1.calc();
  int16_t b = (int16_t)output;
  temp[0]=(uint8_t)(b>>8);
  temp[1]=(uint8_t)(b);
  pid_lun_id_2.ser_error(0.0f-rpm_2);
  output = pid_lun_id_2.calc();
  b = (int16_t)output;
  temp[2]=(uint8_t)(b>>8);
  temp[3]=(uint8_t)(b);
  pid_lun_id_3.ser_error(0.0f-rpm_3);
  output = pid_lun_id_3.calc();
  b = (int16_t)output;
  temp[4]=(uint8_t)(b>>8);
  temp[5]=(uint8_t)(b);
  pid_lun_id_4.ser_error(a-rpm_4);
  output = pid_lun_id_4.calc();
  b = (int16_t)output;
  temp[6]=(uint8_t)(b>>8);
  temp[7]=(uint8_t)(b);
  CAN_Send_Msg(&hcan2,temp,0x200,8);
  CAN_Send_Msg(&hcan1,kong,0x1FE,8);
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
void ModeIwdg(void) {
  uint8_t kong[8]={0,0,0,0,0,0,0,0};
  CAN_Send_Msg(&hcan2,kong,0x200,8);
  CAN_Send_Msg(&hcan1,kong,0x1FE,8);
}
