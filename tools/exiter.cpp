#include "exiter.hpp"

#include <signal.h>
#include <unistd.h>

#include <stdexcept>

namespace tools
{
namespace
{
volatile sig_atomic_t g_exit_requested = 0;
volatile sig_atomic_t g_signal_count = 0;
bool g_exiter_inited = false;

// 只做 async-signal-safe 的操作：置标志；第二次信号强制退出，防止优雅退出卡死。
extern "C" void handle_exit_signal(int)
{
  ++g_signal_count;
  if (g_signal_count >= 2) {
    ::_exit(130);
  }
  g_exit_requested = 1;
}
}  // namespace

Exiter::Exiter()
{
  if (g_exiter_inited) throw std::runtime_error("Multiple Exiter instances!");

  struct sigaction action {};
  action.sa_handler = handle_exit_signal;
  sigemptyset(&action.sa_mask);
  // 处理期间同时屏蔽两种信号，避免嵌套导致计数丢失（第二次信号就无法强制退出）。
  sigaddset(&action.sa_mask, SIGINT);
  sigaddset(&action.sa_mask, SIGTERM);
  // 不设 SA_RESTART：让阻塞的 read / select 返回 EINTR，串口线程能更快看到退出。
  action.sa_flags = 0;

  if (sigaction(SIGINT, &action, nullptr) != 0 || sigaction(SIGTERM, &action, nullptr) != 0) {
    throw std::runtime_error("failed to install signal handlers");
  }
  g_exiter_inited = true;
}

bool Exiter::exit() const { return g_exit_requested != 0; }

bool exit_requested() { return g_exit_requested != 0; }

}  // namespace tools
