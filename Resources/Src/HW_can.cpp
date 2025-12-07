/**
 *******************************************************************************
 * @file      :HW_can.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"
#include "stdint.h"
#include "dipan.hpp"
#include "dm4310_drv.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1, rx_header2;
static uint8_t can1_rx_data[8], can2_rx_data[8];
uint32_t pTxMailbox;
int state1 = 0;
int state2 = 0;
int state3 = 0;
int f_y_axis = 0;
float rc_lv_private = 0;
float rc_lh_private = 0;
float rc_rv_private = 0;
float rc_rh_private = 0;
float rpm = 0;
float rpm_2 = 0;
float rpm_3 = 0;
float rpm_4 = 0;
float rpm_duo_1 = 0;
float rpm_duo_2 = 0;
float rpm_duo_3 = 0;
float rpm_duo_4 = 0;
float angle_duo_1 = 0;
float angle_duo_2 = 0;
float angle_duo_3 = 0;
float angle_duo_4 = 0;
float angle_yaw = 0.0f;
float imu_yaw = 0.0f;
float vel_yaw = 0.0f;
extern Joint_Motor_t yaw_private;
extern dipan::Dipan dipan_private;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief
 * @param        *hcan:
 * @retval       None
 * @note        None
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef canfilter;

  canfilter.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilter.FilterScale = CAN_FILTERSCALE_16BIT;

  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14;
  if (hcan == &hcan1) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterBank = 0;
    canfilter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  } else if (hcan == &hcan2) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterActivation = ENABLE;
    canfilter.FilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  }
}

uint32_t can_rec_times = 0;
uint32_t can_success_times = 0;
uint32_t can_receive_data = 0;

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) ==
        HAL_OK) // 获得接收到的数据头和数据
    {
      
      if (rx_header1.StdId == 0x205) { // 帧头校验
        state1 = 1;
        rpm_duo_1 = (float)(int16_t)(can1_rx_data[2]<<8|can1_rx_data[3]);
        angle_duo_1 = (float)(int16_t)(can1_rx_data[0]<<8|can1_rx_data[1]);                               // 校验通过进行具体数据处理
      }
      if (rx_header1.StdId == 0x206) { // 帧头校验
        rpm_duo_2 = (float)(int16_t)(can1_rx_data[2]<<8|can1_rx_data[3]);
        angle_duo_2 = (float)(int16_t)(can1_rx_data[0]<<8|can1_rx_data[1]);                               // 校验通过进行具体数据处理
      }
      if (rx_header1.StdId == 0x207) { // 帧头校验
        rpm_duo_3 = (float)(int16_t)(can1_rx_data[2]<<8|can1_rx_data[3]);
        angle_duo_3 = (float)(int16_t)(can1_rx_data[0]<<8|can1_rx_data[1]);                               // 校验通过进行具体数据处理
      }
      if (rx_header1.StdId == 0x208) { // 帧头校验
        rpm_duo_4 = (float)(int16_t)(can1_rx_data[2]<<8|can1_rx_data[3]);
        angle_duo_4 = (float)(int16_t)(can1_rx_data[0]<<8|can1_rx_data[1]);                               // 校验通过进行具体数据处理
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 再次使能FIFO0接收中断
}

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {

  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) ==
        HAL_OK) // 获得接收到的数据头和数据
    {
      if(rx_header2.StdId == 0x1FF)
      {
        HAL_IWDG_Refresh(&hiwdg);
        int16_t temp = (int16_t)(((uint16_t)(can2_rx_data[0]) << 8) | ((uint16_t)can2_rx_data[1]));
        rc_lv_private = (float)temp / 1000.0f;
        temp = (int16_t)(((uint16_t)(can2_rx_data[2]) << 8) | ((uint16_t)can2_rx_data[3]));
        rc_lh_private = (float)temp / 1000.0f;
        imu_yaw = (float)((int16_t)(can2_rx_data[4]<<8|can2_rx_data[5])/1000.0f);
        vel_yaw = (float)((int16_t)(can2_rx_data[6]<<8|can2_rx_data[7])/1000.0f);
      }
      if(rx_header2.StdId == 0x0FE)
      {
        rc_rv_private = (float)((int16_t)(can2_rx_data[0]<<8|can2_rx_data[1]))/1000.0f;
        rc_rh_private = (float)((int16_t)(can2_rx_data[2]<<8|can2_rx_data[3]))/1000.0f;
      }
      if (rx_header2.StdId == 0x201) { // 帧头校验
        state2 = 1;
        rpm = (float)(int16_t)(((uint16_t)(can2_rx_data[2]) << 8) | ((uint16_t)can2_rx_data[3]));// 校验通过进行具体数据处理
      }
      if (rx_header2.StdId == 0x202) { // 帧头校验
        state2 = 1;
        rpm_2 = (float)(int16_t)(((uint16_t)(can2_rx_data[2]) << 8) | ((uint16_t)can2_rx_data[3]));// 校验通过进行具体数据处理
      }
      if (rx_header2.StdId == 0x203) { // 帧头校验
        state2 = 1;
        rpm_3 = (float)(int16_t)(((uint16_t)(can2_rx_data[2]) << 8) | ((uint16_t)can2_rx_data[3]));// 校验通过进行具体数据处理
      }
      if (rx_header2.StdId == 0x204) { // 帧头校验
        state2 = 1;
        rpm_4 = (float)(int16_t)(((uint16_t)(can2_rx_data[2]) << 8) | ((uint16_t)can2_rx_data[3]));// 校验通过进行具体数据处理
      }
      if(rx_header2.StdId == 0x11){
        state3 += 1;
        dm4310_fbdata(&yaw_private,can2_rx_data,8);
        angle_yaw = yaw_private.para.pos;
        if(f_y_axis == 0){
          dipan_private.set_angle_raw_y_axis(angle_yaw-1.52);
          f_y_axis = 1;
        }
        else{
          dipan_private.set_angle_now_y_axis(angle_yaw-1.52);
        }
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // 再次使能FIFO0接收中断
}

/**
 * @brief   向can总线发送数据，抄官方的
 * @param   hcan为CAN句柄
 * @param	msg为发送数组首地址
 * @param	id为发送报文
 * @param	len为发送数据长度（字节数）
 * @retval  none
 * @note    主控发送都是len=8字节，再加上帧间隔3位，理论上can总线1ms最多传输9帧
 **/
void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len) {
  CAN_TxHeaderTypeDef TxMessageHeader = {0};
  TxMessageHeader.StdId = id;
  TxMessageHeader.IDE = CAN_ID_STD;
  TxMessageHeader.RTR = CAN_RTR_DATA;
  TxMessageHeader.DLC = len;
  if (HAL_CAN_AddTxMessage(hcan, &TxMessageHeader, msg, &pTxMailbox) !=
      HAL_OK) {
  }
}
