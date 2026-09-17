#include "solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, LARGE_ARMOR_WIDTH / 2, LARGE_ARMOR_HEIGHT / 2},
  {0, -LARGE_ARMOR_WIDTH / 2, LARGE_ARMOR_HEIGHT / 2},
  {0, -LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2},
  {0, LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, SMALL_ARMOR_HEIGHT / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, SMALL_ARMOR_HEIGHT / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2}};

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);

  if (yaml["pnp_max_incidence"]) { // 约束参数，限制装甲板法线与相机光轴的夹角，避免解算出镜像解
    const double max_incidence_degree = yaml["pnp_max_incidence"].as<double>();
    if (max_incidence_degree > 0.0 && max_incidence_degree < 90.0) {
      pnp_max_incidence_ = max_incidence_degree * CV_PI / 180.0;
    } else {
      tools::logger()->warn(
        "[Solver] pnp_max_incidence must be in (0, 90) degrees, use 80 degrees");
    }
  }

  std::array<double, 9> cam_array;
  std::copy_n(camera_matrix_data.begin(), 9, cam_array.begin());
  ba_solver_ = std::make_unique<BaSolver>(cam_array, distort_coeffs_data);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  // q 是电控给出的“初始姿态到当前姿态”旋转，换基到云台系即可，无需取逆。
  const Eigen::Matrix3d R_initial2current =
    q.normalized().toRotationMatrix();

  R_gimbal2world_ =
    R_gimbal2imubody_.transpose() *
    R_initial2current *
    R_gimbal2imubody_; // 作固定基变换
}

//solvePnP（获得姿态）
void Solver::solve(Armor & armor) const
{
  const auto & image_points =
    armor.traditional_points.empty() ? armor.points : armor.traditional_points;
  solve(armor, image_points);
}

void Solver::solve(std::list<Armor> & armors) const
{
  std::map<std::pair<Color, ArmorName>, std::vector<Armor *>> armor_groups;
  for (auto & armor : armors) armor_groups[{armor.color, armor.name}].push_back(&armor);

  for (const auto & [_, armor_group] : armor_groups) {
    const auto use_traditional =
      armor_group.size() == 1 && !armor_group.front()->traditional_points.empty();
    for (auto * armor : armor_group) {
      solve(*armor, use_traditional ? armor->traditional_points : armor->points);
    }
  }
}

void Solver::solve(Armor & armor, const std::vector<cv::Point2f> & image_points) const
{
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // IPPE has two solutions for a planar target. solvePnP() only returns the
  // first one, which can be the mirrored pose under pixel noise. Keep every
  // solution and reject poses whose armor normal cannot face the camera.
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  const int solution_count = cv::solvePnPGeneric(
    object_points, image_points, camera_matrix_, distort_coeffs_, rvecs, tvecs, false,
    cv::SOLVEPNP_IPPE);
  if (solution_count <= 0 || rvecs.size() != tvecs.size()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  const Eigen::Matrix3d R_camera2world = R_gimbal2world_ * R_camera2gimbal_;
  double best_error = std::numeric_limits<double>::infinity();
  cv::Vec3d best_rvec;
  cv::Vec3d best_tvec;
  Eigen::Matrix3d best_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d best_xyz_in_gimbal;
  Eigen::Vector3d best_xyz_in_world;
  double best_yaw = 0.0;
  bool found_valid_solution = false;

  for (std::size_t i = 0; i < rvecs.size(); ++i) { // 遍历所有解，找出最优解
    cv::Mat rvec_mat;
    cv::Mat tvec_mat;
    rvecs[i].reshape(1, 3).convertTo(rvec_mat, CV_64F);
    tvecs[i].reshape(1, 3).convertTo(tvec_mat, CV_64F);
    const cv::Vec3d candidate_rvec(
      rvec_mat.at<double>(0, 0), rvec_mat.at<double>(1, 0), rvec_mat.at<double>(2, 0));
    const cv::Vec3d candidate_tvec(
      tvec_mat.at<double>(0, 0), tvec_mat.at<double>(1, 0), tvec_mat.at<double>(2, 0));
    if (!cv::checkRange(candidate_rvec) || !cv::checkRange(candidate_tvec) ||
        candidate_tvec[2] <= 0.0) { // 排除无效解
      continue;
    }

    const Eigen::Vector3d xyz_in_camera(
      candidate_tvec[0], candidate_tvec[1], candidate_tvec[2]);
    const Eigen::Vector3d xyz_in_gimbal =
      R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    const Eigen::Vector3d xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
    if (!xyz_in_world.allFinite() || xyz_in_world.head<2>().norm() < 1e-9) continue;

    cv::Mat rmat;
    cv::Rodrigues(candidate_rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    const Eigen::Matrix3d R_armor2world = R_camera2world * R_armor2camera;
    // Local +X is the armor normal: upward (+pitch) has a negative world Z
    // component; the downward outpost pose has a positive one.
    const double normal_z = R_armor2world(2, 0);
    if (armor.name == ArmorName::outpost ? normal_z < 0.0 : normal_z > 0.0) continue;
    const double yaw = std::atan2(R_armor2world(1, 0), R_armor2world(0, 0));
    const double bearing = std::atan2(xyz_in_world.y(), xyz_in_world.x()); // 世界系方位角
    const double incidence = std::abs(tools::limit_rad(yaw - bearing)); // 装甲板法线与相机光轴夹角
    if (!std::isfinite(incidence) || incidence > pnp_max_incidence_) continue;

    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points, candidate_rvec, candidate_tvec, camera_matrix_, distort_coeffs_,
      reprojected_points);
    if (reprojected_points.size() != image_points.size()) continue;
    double reprojection_error = 0.0;
    for (std::size_t j = 0; j < image_points.size(); ++j) { // 误差计算方法：所有点的重投影误差平均值
      reprojection_error += cv::norm(image_points[j] - reprojected_points[j]);
    }
    reprojection_error /= image_points.size();
    if (!std::isfinite(reprojection_error) || reprojection_error >= best_error) continue;

    best_error = reprojection_error;
    best_rvec = candidate_rvec;
    best_tvec = candidate_tvec;
    best_R = R_armor2camera;
    best_xyz_in_gimbal = xyz_in_gimbal;
    best_xyz_in_world = xyz_in_world;
    best_yaw = yaw;
    found_valid_solution = true;
  }

  if (!found_valid_solution) {
    armor.name = ArmorName::not_armor;
    return;
  }

  // 每块装甲板的中心：相机 -> 云台 -> 世界系。
  armor.xyz_in_gimbal = best_xyz_in_gimbal;
  armor.xyz_in_world = best_xyz_in_world;
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  armor.yaw_raw = best_yaw;

  Eigen::Matrix3d R = best_R;
  const Eigen::Vector3d t(best_tvec[0], best_tvec[1], best_tvec[2]);
  const Eigen::Matrix3d R_armor2world_pnp = R_camera2world * R;

  const double armor_roll = rotationMatrixToRPY(R_armor2world_pnp)[0] * 180 / M_PI;

  Eigen::Matrix3d R_armor2camera = R;
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  // First find the global basin with the bounded grid search, then let BA
  // refine from that yaw. Keep the grid result if BA leaves the valid basin or
  // increases the reprojection error.
  Armor grid_result = armor;
  optimize_yaw(grid_result, image_points);
  double grid_yaw = grid_result.ypr_in_world[0];
  double grid_error = armor_reprojection_error(grid_result, image_points, grid_yaw, 0.0);
  const double raw_error = armor_reprojection_error(armor, image_points, best_yaw, 0.0);
  if (raw_error < grid_error) {
    grid_yaw = best_yaw;
    grid_error = raw_error;
  }

  if (!use_ba_optimization(armor, armor_roll)) { // 排除roll过大或平衡装甲的情况
    armor.ypr_in_world[0] = grid_yaw;
    return;
  }

  const Eigen::Matrix3d R_ba = // BA优化后的旋转矩阵（世界系）
    ba_solver_->solveBa(armor, image_points, t, R, R_camera2world, grid_yaw);
  const Eigen::Matrix3d R_ba_in_gimbal = R_camera2gimbal_ * R_ba;
  const Eigen::Matrix3d R_ba_in_world = R_gimbal2world_ * R_ba_in_gimbal;
  const Eigen::Vector3d ba_ypr_in_world = tools::eulers(R_ba_in_world, 2, 1, 0); // BA优化后的姿态（世界系）
  const double ba_yaw = ba_ypr_in_world[0];
  const double ba_incidence =
    std::abs(tools::limit_rad(ba_yaw - armor.ypd_in_world[0]));
  const double ba_error = armor_reprojection_error(armor, image_points, ba_yaw, 0.0);

  if (R_ba.allFinite() && ba_ypr_in_world.allFinite() && std::isfinite(ba_error) &&
      ba_incidence <= pnp_max_incidence_ && ba_error <= grid_error + 1e-6) {
    armor.ypr_in_gimbal = tools::eulers(R_ba_in_gimbal, 2, 1, 0);
    armor.ypr_in_world = ba_ypr_in_world;
  } else {
    // 如果BA优化失败，使用网格搜索的结果
    armor.ypr_in_world[0] = grid_yaw;
  }
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -ARMOR_PITCH_RAD : ARMOR_PITCH_RAD;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // solve
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

void Solver::optimize_yaw(Armor & armor, const std::vector<cv::Point2f> & image_points) const
{
  constexpr double SEARCH_RANGE = 140;  // degree
  // Center the visibility prior on the observed line of sight. This remains
  // correct when the gimbal convention is X-right/Y-forward.
  auto yaw0 =
    tools::limit_rad(armor.ypd_in_world[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(
      armor, image_points, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.ypr_in_world[0] = best_yaw;
}

bool Solver::use_ba_optimization(const Armor & armor, double armor_roll) const
{
  if (!ba_solver_) return false;

  // 平衡装甲的姿态假设更不稳定，先跳过 BA
  const bool is_balance = (armor.type == ArmorType::big) &&
                          (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                           armor.name == ArmorName::five);
  if (is_balance) return false;

  return abs(armor_roll) < 15.0;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, const std::vector<cv::Point2f> & image_points, double yaw,
  const double & inclined) const
{
  auto reprojected_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(image_points[i] - reprojected_points[i]);
  // auto error = SJTU_cost(reprojected_points, image_points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}


Eigen::Vector3d Solver::rotationMatrixToRPY(const Eigen::Matrix3d &R) const {
  Eigen::Vector3d rpy;
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), tf2 getRPY order: roll, pitch, yaw
  rpy[0] = std::atan2(R(2, 1), R(2, 2));                                          // roll
  rpy[1] = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))); // pitch
  rpy[2] = std::atan2(R(1, 0), R(0, 0));                                          // yaw
  return rpy;
}

Eigen::MatrixXd Solver::cvToEigen(const cv::Mat &cv_mat) const {
  Eigen::MatrixXd eigen_mat = Eigen::MatrixXd::Zero(cv_mat.rows, cv_mat.cols);
  cv::cv2eigen(cv_mat, eigen_mat);
  return eigen_mat;
}
}  // namespace auto_aim
