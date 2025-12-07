#ifndef YAW_HPP
#define YAW_HPP


namespace yaw{

class Yaw{
private:
float purpose_yaw_ = 0.0f;
float now_yaw_ = 0.0f;
public:
void set_imu_now_yaw(float yaw){
    now_yaw_ = yaw;
}
void set_purpose_yaw(float yaw){
    purpose_yaw_ = yaw;
}
float calc_yaw_error(void);
};

}//yaw










#endif // YAW_HPP