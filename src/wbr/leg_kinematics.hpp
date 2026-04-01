#ifndef WBR_LEG_KINEMATICS_HPP_
#define WBR_LEG_KINEMATICS_HPP_

#include <arm_math.h>
#include <librm.hpp>

using namespace rm;

namespace wbr {

class LegKinematics {
public:
  LegKinematics(f32 l1, f32 l2);

  void Update(f32 dt = 0.002f);

  void SetPhi1(const f32 phi1) { this->phi1_ = phi1; }
  void SetPhi4(const f32 phi4) { this->phi4_ = phi4; }
  void SetWPhi1(const f32 w_phi1) { this->w_phi1_ = w_phi1; }
  void SetWPhi4(const f32 w_phi4) { this->w_phi4_ = w_phi4; }

  [[nodiscard]] f32 l0() const { return this->l0_; }
  [[nodiscard]] f32 l0_dot() const { return this->l0_dot_; }
  [[nodiscard]] f32 phi0() const { return this->phi0_; }
  [[nodiscard]] f32 beta() const { return this->beta_; }
  [[nodiscard]] f32 beta_dot() const { return this->beta_dot_; }
  [[nodiscard]] f32 xc() const { return this->xc_; }
  [[nodiscard]] f32 yc() const { return this->yc_; }
  [[nodiscard]] f32 xc_dot() const { return this->xc_dot_; }
  [[nodiscard]] f32 yc_dot() const { return this->yc_dot_; }
  [[nodiscard]] f32 jacobi_00() const { return this->jacobi_00_; }
  [[nodiscard]] f32 jacobi_01() const { return this->jacobi_01_; }
  [[nodiscard]] f32 jacobi_10() const { return this->jacobi_10_; }
  [[nodiscard]] f32 jacobi_11() const { return this->jacobi_11_; }

private:
  f32 phi1_{0.0f}, phi4_{0.0f};
  f32 beta_{0.0f}, beta_last{0.0f};
  f32 beta_dot_{0.0f};
  f32 w_phi1_{0.0f}, w_phi4_{0.0f};
  f32 phi2_{0.0f}, phi3_{0.0f};
  f32 w_phi2_{0.0f}, w_phi3_{0.0f};

  f32 l0_{0.0f}, phi0_{0.0f};
  f32 l0_dot_{0.0f};

  // 中间变量
  f32 A0_{0.0f}, B0_{0.0f}, C0_{0.0f};
  f32 xb_{0.0f}, yb_{0.0f};
  f32 xd_{0.0f}, yd_{0.0f};
  f32 xc_{0.0f}, yc_{0.0f};
  f32 l_bd_{0.0f};
  f32 xc_dot_{0.0f}, yc_dot_{0.0f};

  // 机械参数
  f32 l1_;
  f32 l2_;

  // 雅可比行列式
  f32 jacobi_00_{0.0f}, jacobi_01_{0.0f}, jacobi_10_{0.0f}, jacobi_11_{0.0f};
};

} // namespace wbr

#endif // WBR_LEG_KINEMATICS_HPP_
