#ifndef VMC_HPP
#define VMC_HPP

#include "librm.hpp"

#include "arm_math.h"

using namespace rm;

class Vmc {
 public:
  Vmc(const f32 l1, const f32 l2, const f32 l5);

  void Update();

  void SetPhi1(const f32 phi1) { this->phi1_ = phi1; }
  void SetPhi4(const f32 phi4) { this->phi4_ = phi4; }
  void SetWPhi1(const f32 w_phi1) { this->w_phi1_ = w_phi1; }
  void SetWPhi4(const f32 w_phi4) { this->w_phi4_ = w_phi4; }

  [[nodiscard]] const f32 l0() { return this->l0_; }
  [[nodiscard]] const f32 phi0() { return this->phi0_; }
  [[nodiscard]] const f32 xc() { return this->xc_; }
  [[nodiscard]] const f32 yc() { return this->yc_; }
  [[nodiscard]] const f32 xc_dot() { return this->xc_dot_; }
  [[nodiscard]] const f32 yc_dot() { return this->yc_dot_; }
  [[nodiscard]] const f32 jacobi_00() { return this->jacobi_00_; }
  [[nodiscard]] const f32 jacobi_01() { return this->jacobi_01_; }
  [[nodiscard]] const f32 jacobi_10() { return this->jacobi_10_; }
  [[nodiscard]] const f32 jacobi_11() { return this->jacobi_11_; }

 private:
  f32 phi1_, phi4_;
  f32 w_phi1_, w_phi4_;
  f32 phi2_, phi3_;
  f32 w_phi2_, w_phi3_;

  f32 l0_, phi0_;

  // 中间变量
  f32 A0_, B0_, C0_;
  f32 xb_, yb_;
  f32 xd_, yd_;
  f32 xc_, yc_;
  f32 l_bd_;
  f32 xc_dot_, yc_dot_;

  // 机械参数
  f32 l1_;
  f32 l2_;
  f32 l5_;

  // vmc相关
  f32 jacobi_00_, jacobi_01_, jacobi_10_, jacobi_11_;
};

#endif