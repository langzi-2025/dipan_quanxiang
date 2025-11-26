#ifndef DUO_HPP
#define DUO_HPP
namespace duo{
class Duo{
  public:
  Duo(float a){
    original_angle_ = data_to_rad(a);
  }
  void set_now_angle(float angle_raw);
  void set_purpose_angle(float angle_raw)
  {
    purpose_angle_ = angle_raw;
  }
  void calc_error(void);
  float data_to_rad(float angle_raw);
  float get_error(void)
  {
    return error_;
  }
  private:
  float original_angle_;
  float now_angle_;
  float purpose_angle_;
  float error_ = 0.0f;
};
}





#endif // DUO_HPP