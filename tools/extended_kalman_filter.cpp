#include "extended_kalman_filter.hpp"

#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nis_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
  data["update_accepted"] = 0.0;
  data["measurement_dim"] = 0.0;
  for (const auto * key :
       {"residual_yaw_2", "residual_pitch_2", "residual_distance_2", "residual_angle_2"})
    data[key] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q; // 预测协方差矩阵
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(
    z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  // 创新必须在先验状态处计算，不能使用已经被观测拉近的后验残差。
  const Eigen::VectorXd residual = z_subtract(z, h(x));
  // 计算创新协方差矩阵
  const Eigen::MatrixXd S = H * P * H.transpose() + R;
  Eigen::LDLT<Eigen::MatrixXd> decomposition(S);
  const bool valid = residual.allFinite() && S.allFinite() &&
                     decomposition.info() == Eigen::Success &&
                     (decomposition.vectorD().array() > 0).all();
  last_nis =
    valid ? residual.dot(decomposition.solve(residual)) : std::numeric_limits<double>::infinity();
  const bool failed = !std::isfinite(last_nis) || last_nis > nis_threshold(z.size());
  // 95% 门限用于统计；更宽松的关联门限用于拒绝明显异常观测，两者含义不同。
  recent_nis_failures.push_back(failed ? 1 : 0);
  if (recent_nis_failures.size() > window_size) recent_nis_failures.pop_front();
  data["nis"] = last_nis;
  data["nis_fail"] = failed ? 1.0 : 0.0;
  data["measurement_dim"] = z.size();
  data["recent_nis_failures"] = static_cast<double>(std::accumulate(
                                  recent_nis_failures.begin(), recent_nis_failures.end(), 0)) /
                                std::max<size_t>(1, recent_nis_failures.size());
  const char * keys[] = {"residual_yaw",        "residual_pitch",  "residual_distance",
                         "residual_angle",      "residual_yaw_2",  "residual_pitch_2",
                         "residual_distance_2", "residual_angle_2"};
  for (int i = 0; i < 8; ++i) data[keys[i]] = i < residual.size() ? residual[i] : 0.0;
  last_update_accepted = false;
  data["update_accepted"] = 0.0;
  if (
    !valid || !std::isfinite(last_nis) ||
    (nis_gate_probability > 0 && last_nis > nis_threshold(z.size(), nis_gate_probability)))
    return x;  // 拒绝时保留先验 x/P，交给 tracker 的丢失状态机处理。

  const Eigen::MatrixXd K = decomposition.solve((P * H.transpose()).transpose()).transpose(); // 计算卡尔曼增益
  const Eigen::VectorXd correction = K * residual; // 计算状态修正量
  if (!correction.allFinite()) return x;
  const Eigen::VectorXd posterior_x = x_add(x, correction); // 更新状态向量
  const Eigen::MatrixXd A = I - K * H; // 计算后验协方差矩阵
  // Joseph 形式支持固定参数造成的半正定 P，不需要对 P 求逆。
  const Eigen::MatrixXd posterior_P = A * P * A.transpose() + K * R * K.transpose(); // 更新协方差矩阵
  if (!posterior_x.allFinite() || !posterior_P.allFinite()) return x;
  x = posterior_x;
  P = (posterior_P + posterior_P.transpose()) * 0.5; // 确保协方差矩阵是对称的
  last_update_accepted = true;
  data["update_accepted"] = 1.0;
  // 无真实状态时不能计算 NEES；状态修正量也不能冒充真实估计误差。
  return x;
}

double ExtendedKalmanFilter::nis_threshold(int dimension, double probability)
{
  return boost::math::quantile(boost::math::chi_squared(dimension), probability); // 计算卡方分布的分位点
}

double ExtendedKalmanFilter::innovation_nis(
  const Eigen::VectorXd & residual, const Eigen::MatrixXd & H, const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & R)
{
  const Eigen::MatrixXd S = H * P * H.transpose() + R; // 计算创新协方差矩阵
  if (!S.allFinite() || !residual.allFinite()) return std::numeric_limits<double>::infinity();
  Eigen::LDLT<Eigen::MatrixXd> decomposition(S);
  if (decomposition.info() != Eigen::Success || !(decomposition.vectorD().array() > 0).all())
    return std::numeric_limits<double>::infinity();
  // 计算创新的归一化平方误差
  const double value = residual.dot(decomposition.solve(residual));
  return std::isfinite(value) ? value : std::numeric_limits<double>::infinity();
}

}  // namespace tools