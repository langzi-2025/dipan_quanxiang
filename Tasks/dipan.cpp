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
      v_[i]=sqrt(vx_temp[i]*vx_temp[i]+vy_temp[i]*vy_temp[i])/3.14*60/104.0f;
      if(abs(v_[i])>1959.36)
      {
        if(v_[i]>0)
        {
            v_[i] = 1959.36;
        }
        else
        {
            v_[i] = -1959.36;
        }
      }
      float angle_temp = 0.0f;
      if(vy_temp[i]=0)
      {
        if(vx_temp[i]>0)
        {
            angle_temp = 3.14/2.0f;
        }
        else if(vx_temp[i]<0)
        {
            angle_temp = -3.14/2.0f;
        }
        else
        {
            angle_temp = 0.0f;
        }
      }
      else if(vy_temp[i]>0)
      {
        angle_temp = atan(vx_temp[i]/vy_temp[i]);
      }
      else if(vy_temp[i]<0)
      {
        angle_temp = -atan(vx_temp[i]/vy_temp[i])+3.14;
      }
      angle_[i] = angle_temp;
    }
}
}