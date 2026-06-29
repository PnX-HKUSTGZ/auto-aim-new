#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <chrono>
#include <condition_variable>
#include <list>
#include <mutex>
#include <optional>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/plotter.hpp"

namespace auto_aim
{
namespace multithread
{

class CommandGener
{
public:
  CommandGener(
    auto_aim::Planner & planner, io::Gimbal & gimbal, tools::Plotter & plotter,
    bool debug = false);

  ~CommandGener();

  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
    double bullet_speed);

private:
  struct Input
  {
    std::optional<auto_aim::Target> target;
    std::chrono::steady_clock::time_point t;
    double bullet_speed;
  };

  io::Gimbal * gimbal_;
  auto_aim::Planner & planner_;
  tools::Plotter & plotter_;

  std::optional<Input> latest_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread thread_;
  bool stop_, debug_;

  void generate_command();
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM_MULTITHREAD__HPP