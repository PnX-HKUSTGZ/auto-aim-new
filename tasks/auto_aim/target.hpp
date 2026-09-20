#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  std::vector<int> last_id;  // debug only, 上一帧各装甲板匹配到的板号（单板 1 个 / 双板 2 个）

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h1);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  bool update_armors(const std::list<Armor> & armors);
  bool bad_quality() const;

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_ = 4;
  int switch_count_ = 0;
  int update_count_ = 0;

  bool is_switch_ = false, is_converged_ = false;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  std::list<std::pair<Armor, int>> match_armor_id(const std::list<Armor> & armors);

  // 单板、双板与关联使用同一观测模型，避免噪声和角度处理不一致。
  Eigen::Vector4d observation(const Eigen::VectorXd & x, int id) const;
  Eigen::Matrix4d measurement_noise(const Armor & armor) const;
  Eigen::Vector4d innovation(const Armor & armor, int id) const;
  bool update_ypda(const std::list<std::pair<Armor, int>> & matched_armors);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
