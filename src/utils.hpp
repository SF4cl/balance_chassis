
#include "librm.hpp"

using namespace rm;

class FirstOrderFilter {
 public:
  FirstOrderFilter(f32 frame_period, f32 num) {
    frame_period_ = frame_period;
    num_ = num;
    input_ = 0;
    out_ = 0;
  }

  [[nodiscard]] f32 value() { return out_; }

  void Update(f32 input) {
    input_ = input;
    out_ = num_ / (num_ + frame_period_) * out_ + frame_period_ / (num_ + frame_period_) * input;
  }

 private:
  f32 input_;
  f32 out_;
  f32 num_;
  f32 frame_period_;
};

class UserPid {
 public:
  UserPid(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_out_ = max_out;
    max_iout_ = max_iout;
    min_out_ = -max_out;
    min_iout_ = -max_iout;
  }

  void SetIout(f32 iout) { Iout_ = iout; }

  void Update(f32 set, f32 ref) {
    error_[2] = error_[1];
    error_[1] = error_[0];
    set_ = set;
    fdb_ = ref;
    error_[0] = set - ref;
    Pout_ = kp_ * error_[0];
    Iout_ += ki_ * error_[0];
    Dbuf_[2] = Dbuf_[1];
    Dbuf_[1] = Dbuf_[0];
    Dbuf_[0] = (error_[0] - error_[1]);
    Dout_ = kd_ * Dbuf_[0];
    LimitMax(Iout_, max_iout_);
    LimitMin(Iout_, min_iout_);
    out_ = Pout_ + Iout_ + Dout_;
    LimitMax(out_, max_out_);
    LimitMin(out_, min_out_);
  }

  [[nodiscard]] f32 value() { return out_; }

 private:
  f32 kp_;
  f32 ki_;
  f32 kd_;
  f32 max_out_;
  f32 max_iout_;
  f32 min_out_;
  f32 min_iout_;
  f32 out_;
  f32 error_[3];
  f32 set_, fdb_;
  f32 Pout_, Iout_, Dout_;
  f32 Dbuf_[3];

  void LimitMax(f32 &val, f32 max) {
    if (val > max) {
      val = max;
    }
  }
  void LimitMin(f32 &val, f32 min) {
    if (val < min) {
      val = min;
    }
  }
};