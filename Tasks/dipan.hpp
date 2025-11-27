#ifndef DIPAN_HPP
#define DIPAN_HPP

namespace dipan
{
class Dipan{
public:
void calc_jiesuan(void);
private:
float v_[4]={0.0f,0.0f,0.0f,0.0f};
float angle_[4]={0.0f,0.0f,0.0f,0.0f};
float v_x_ = 0.0f;
float v_y_ = 0.0f;
float w_ = 0.0f;
};
}






#endif // DIPAN_HPP