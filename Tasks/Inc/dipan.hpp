#ifndef DIPAN_HPP
#define DIPAN_HPP
#include <math.h>
namespace dipan
{
class Dipan{
public:
void calc_jiesuan(void);
void set_vx(float vx)
{
  v_x_ = vx;
}
void set_vy(float vy)
{
  v_y_ = vy;
}
void set_w(float w)
{
  w_ = w;
}
void get_v(float a[])
{
  a[0] = v_[0];
  a[1] = v_[1];
  a[2] = v_[2];
  a[3] = v_[3];
}
void get_angle(float a[])
{
  a[0] = angle_[0];
  a[1] = angle_[1];
  a[2] = angle_[2];
  a[3] = angle_[3];
}
void set_angle_raw_y_axis(float angle_raw_y_axis)
{
  this->angle_raw_y_axis = angle_raw_y_axis;
}
void set_angle_now_y_axis(float angle_now_y_axis)
{
  this->angle_now_y_axis = angle_now_y_axis;
}
private:
float v_[4]={0.0f,0.0f,0.0f,0.0f};
float angle_[4]={0.0f,0.0f,0.0f,0.0f};
float v_x_ = 0.0f;
float v_y_ = 0.0f;
float w_ = 0.0f;
float angle_raw_y_axis = 0.0f;
float angle_now_y_axis = 0.0f;
};
}






#endif // DIPAN_HPP