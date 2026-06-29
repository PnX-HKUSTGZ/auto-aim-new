#include "commandgener.hpp"

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{
CommandGener::CommandGener(
  auto_aim::Planner & planner, io::Gimbal & gimbal, tools::Plotter & plotter, bool debug)
: gimbal_(&gimbal),
  planner_(planner),
  plotter_(plotter),
  stop_(false),
  debug_(debug)
{
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  if (gimbal_) gimbal_->send(false, false, 0, 0, 0, 0, 0, 0);
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets.empty() ? std::nullopt : std::make_optional(targets.front()), t, bullet_speed};
  cv_.notify_one();
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();
  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;
    }
    if (input) {
      auto plan = planner_.plan(input->target, input->bullet_speed);
      auto horizon_distance = input->target.has_value()
                                ? std::sqrt(
                                    tools::square(input->target->ekf_x()[0]) +
                                    tools::square(input->target->ekf_x()[2]))
                                : 0;

      gimbal_->send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["target_yaw"] = plan.target_yaw;
        data["target_pitch"] = plan.target_pitch;
        data["plan_yaw"] = plan.yaw;
        data["plan_yaw_vel"] = plan.yaw_vel;
        data["plan_yaw_acc"] = plan.yaw_acc;
        data["plan_pitch"] = plan.pitch;
        data["plan_pitch_vel"] = plan.pitch_vel;
        data["plan_pitch_acc"] = plan.pitch_acc;
        data["fire"] = plan.fire ? 1 : 0;
        data["horizon_distance"] = horizon_distance;
        plotter_.plot(data);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  //approximately 500Hz
  }
}

}  // namespace multithread

}  // namespace auto_aim