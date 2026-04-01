#include "wbr/leg_kinematics.hpp"
#include <cmath>

namespace wbr {

LegKinematics::LegKinematics(const f32 l1, const f32 l2) : l1_(l1), l2_(l2) {}

void LegKinematics::Update(const f32 dt) {
  xb_ = l1_ * arm_cos_f32(phi1_);
  yb_ = l1_ * arm_sin_f32(phi1_);
  xd_ = l1_ * arm_cos_f32(phi4_);
  yd_ = l1_ * arm_sin_f32(phi4_);
  l_bd_ = std::hypot(xd_ - xb_, yd_ - yb_);

  A0_ = 2.0f * l2_ * (xd_ - xb_);
  B0_ = 2.0f * l2_ * (yd_ - yb_);
  C0_ = l_bd_ * l_bd_;
  phi2_ = 2.0f * std::atan2(B0_ + std::sqrt(A0_ * A0_ + B0_ * B0_ - C0_ * C0_),
                            A0_ + C0_);

  beta_ = phi2_ + PI - phi1_;
  beta_dot_ = (beta_ - beta_last) / dt;
  beta_last = beta_;
  phi3_ = std::acos((xb_ + l2_ * arm_cos_f32(phi2_) - xd_) / l2_);

  xc_ = xb_ + l2_ * arm_cos_f32(phi2_);
  yc_ = yb_ + l2_ * arm_sin_f32(phi2_);

  phi0_ = std::atan2(yc_, xc_);
  l0_ = std::hypot(xc_, yc_);

  const f32 sin_p12 = arm_sin_f32(phi1_ - phi2_);
  const f32 sin_p34 = arm_sin_f32(phi3_ - phi4_);
  const f32 sin_p23 = arm_sin_f32(phi2_ - phi3_);
  const f32 sin_p32 = arm_sin_f32(phi3_ - phi2_);
  const f32 sin_p03 = arm_sin_f32(phi0_ - phi3_);
  const f32 cos_p03 = arm_cos_f32(phi0_ - phi3_);
  const f32 sin_p02 = arm_sin_f32(phi0_ - phi2_);
  const f32 cos_p02 = arm_cos_f32(phi0_ - phi2_);

  const f32 sin_p2 = arm_sin_f32(phi2_);
  const f32 cos_p2 = arm_cos_f32(phi2_);
  const f32 sin_p3 = arm_sin_f32(phi3_);
  const f32 cos_p3 = arm_cos_f32(phi3_);

  xc_dot_ = l1_ * sin_p12 * sin_p3 / sin_p23 * w_phi1_ +
            l1_ * sin_p34 * sin_p2 / sin_p23 * w_phi4_;

  yc_dot_ = l1_ * sin_p12 * cos_p3 / sin_p23 * w_phi1_ -
            l1_ * sin_p34 * cos_p2 / sin_p23 * w_phi4_;

  jacobi_00_ = l1_ * sin_p03 * sin_p12 / sin_p32;
  jacobi_01_ = l1_ * cos_p03 * sin_p12 / (l0_ * sin_p32);
  jacobi_10_ = l1_ * sin_p02 * sin_p34 / sin_p32;
  jacobi_11_ = l1_ * cos_p02 * sin_p34 / (l0_ * sin_p32);

  l0_dot_ = (std::hypot(xc_, yc_) - l0_) / dt;
}

} // namespace wbr
