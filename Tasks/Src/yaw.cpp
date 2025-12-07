#include "yaw.hpp"
namespace yaw{
float Yaw::calc_yaw_error(void){
    float yaw_error = purpose_yaw_-now_yaw_;
    if(yaw_error >= 3.14){
        yaw_error -= 6.28;
    }
    if(yaw_error <= -3.14){
        yaw_error += 6.28;
    }

    return yaw_error;
}
}