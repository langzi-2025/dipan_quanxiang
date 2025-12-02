/*
 * @Author: rogue-wave zhangjingjie@zju.edu.cn
 * @Date: 2025-11-25 22:07:37
 * @LastEditors: rogue-wave zhangjingjie@zju.edu.cn
 * @LastEditTime: 2025-11-25 23:06:09
 * @FilePath: \dipan_quanxiang\Tasks\duo.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "duo.hpp"
namespace duo{
void Duo::set_now_angle(float angle_raw){
now_angle_ =  data_to_rad(angle_raw) - original_angle_;
if(now_angle_ < 0 ){
  now_angle_+=2*3.14;
}
}
void Duo::calc_error(void)
{
  error_ = purpose_angle_ - now_angle_;
  if(error_ >= 3.14)
  {
    error_ -= 2*3.14;
  }
  else if(error_ <= -3.14)
  {
    error_ += 2*3.14;
  }
}
float Duo::data_to_rad(float angle_raw)
{
  return angle_raw/8191.0f*2*3.14;
}
}