/*
 * @Author: rogue-wave zhangjingjie@zju.edu.cn
 * @Date: 2025-11-27 23:20:46
 * @LastEditors: rogue-wave zhangjingjie@zju.edu.cn
 * @LastEditTime: 2025-11-27 23:24:25
 * @FilePath: \dipan_quanxiang\Tasks\dipan.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "dipan.hpp"
#include <math.h>
namespace dipan {
static constexpr float raw_angle = 0.678f;
static constexpr float len = 246.0f;
/**
 * @brief       底盘解算，输入单位：mm/s，输出单位：rpm
 * @retval        无
 * @note        注意监测atan的路基正确性
 * 底盘解算完成
 */
void dipan::Dipan::calc_jiesuan(void)
{
    float vx_temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float vy_temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    vx_temp[0] = v_x_-w_*len*cos(raw_angle);
    vy_temp[0] = v_y_-w_*len*sin(raw_angle);
    vx_temp[1] = v_x_+w_*len*cos(raw_angle);
    vy_temp[1] = v_y_-w_*len*sin(raw_angle);
    vx_temp[2] = v_x_+w_*len*cos(raw_angle);
    vy_temp[2] = v_y_+w_*len*sin(raw_angle);
    vx_temp[3] = v_x_-w_*len*cos(raw_angle);
    vy_temp[3] = v_y_+w_*len*sin(raw_angle);
    for(int i = 0;i<4;i++)
    {
      v_[i]=sqrt(vx_temp[i]*vx_temp[i]+vy_temp[i]*vy_temp[i])*60.0f/(3.14*104.0f)*20.0f;
      angle_[i] = atan2(vx_temp[i],vy_temp[i]);
      if(angle_[i]<0)
      {
        angle_[i] += 2*3.14;
      }
    }
    if(abs(v_x_/1900.0f) < 0.05f && abs(v_y_/1900.0f) < 0.05f && abs(w_)<0.05f)
    {
      angle_[0] = 3.14/4.0f;
      angle_[1] = 2*3.14-3.14/4.0f;
      angle_[2] = 3.14/4.0f;
      angle_[3] = 2*3.14-3.14/4.0f;
    }
  }
}