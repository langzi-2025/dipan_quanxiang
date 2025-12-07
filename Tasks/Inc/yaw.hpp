#ifndef YAW_HPP
#define YAW_HPP

#include <math.h>
namespace yaw{

class Yaw{
private:
float purpose_yaw_ = 0.0f;
float now_yaw_ = 0.0f;
int yaw_angle_state_ = 0;
public:
void set_imu_now_yaw(float yaw){
    now_yaw_ = yaw;
}
void set_purpose_yaw(float rc_rh){
    if(yaw_angle_state_ == 0){
        purpose_yaw_ = now_yaw_;
        yaw_angle_state_ = 1;
    }
    else{
        if(fabs(rc_rh)<0.1f)
        {
            purpose_yaw_ += 0.0f;
        }
        else{
        purpose_yaw_ += rc_rh*0.004f;
        if(purpose_yaw_ >= 3.14f){
            purpose_yaw_ -= 6.28f;
        }
        else if(purpose_yaw_ <= -3.14f){
            purpose_yaw_ += 6.28f;
        }
        }
    }
}
float calc_yaw_error(void);
};
}//yaw










#endif // YAW_HPP