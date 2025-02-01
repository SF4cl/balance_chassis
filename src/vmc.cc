#include "vmc.hpp"

#include "math.h"

Vmc::Vmc(const f32 l1, const f32 l2, const f32 l5) : l1_(l1), l2_(l2), l5_(l5) {}

void Vmc::Update() {
  xb_ = l1_ * arm_cos_f32(phi1_) - l5_ / 2.f;
  yb_ = l1_ * arm_sin_f32(phi1_);
  xd_ = l5_ / 2.f + l1_ * arm_cos_f32(phi4_);
  yd_ = l1_ * arm_sin_f32(phi4_);
  l_bd_ = sqrtf((xd_ - xb_) * (xd_ - xb_) + (yd_ - yb_) * (yd_ - yb_));

  A0_ = 2 * l2_ * (xd_ - xb_);
  B0_ = 2 * l2_ * (yd_ - yb_);
  C0_ = l_bd_ * l_bd_;
  phi2_ = 2 * atan2f((B0_ + sqrtf(A0_ * A0_ + B0_ * B0_ - C0_ * C0_)), (A0_ + C0_));
  phi3_ = atan2f(yb_ - yd_ + l2_ * arm_sin_f32(phi2_), xb_ - xd_ + l2_ * arm_cos_f32(phi2_));

  xc_ = xb_ + l2_ * arm_cos_f32(phi2_);
  yc_ = yb_ + l2_ * arm_sin_f32(phi2_);

  phi0_ = atan2f(yc_, xc_);
  l0_ = sqrtf(xc_ * xc_ + yc_ * yc_);

  xc_dot_ = l1_ * sinf(phi1_ - phi2_) * sinf(phi3_) / sinf(phi2_ - phi3_) * w_phi1_ +
            l1_ * sinf(phi3_ - phi4_) * sinf(phi2_) / sinf(phi2_ - phi3_) * w_phi4_;
  yc_dot_ = l1_ * sinf(phi1_ - phi2_) * cosf(phi3_) / sinf(phi2_ - phi3_) * w_phi1_ -
            l1_ * sinf(phi3_ - phi4_) * cosf(phi2_) / sinf(phi2_ - phi3_) * w_phi4_;

  jacobi_00_ = l1_ * arm_sin_f32(phi0_ - phi3_) * arm_sin_f32(phi1_ - phi2_) / arm_sin_f32(phi3_ - phi2_);
  jacobi_01_ = l1_ * arm_cos_f32(phi0_ - phi3_) * arm_sin_f32(phi1_ - phi2_) / (l0_ * arm_sin_f32(phi3_ - phi2_));
  jacobi_10_ = l1_ * arm_sin_f32(phi0_ - phi2_) * arm_sin_f32(phi3_ - phi4_) / arm_sin_f32(phi3_ - phi2_);
  jacobi_11_ = l1_ * arm_cos_f32(phi0_ - phi2_) * arm_sin_f32(phi3_ - phi4_) / (l0_ * arm_sin_f32(phi3_ - phi2_));
}