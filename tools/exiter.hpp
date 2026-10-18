#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
// 统一管理退出请求：SIGINT / SIGTERM 只置标志，清理交给 main 与析构函数。
class Exiter
{
public:
  Exiter();

  bool exit() const;
};

// 供底层阻塞等待（串口 / 相机队列）轮询，避免信号被 condition_variable 吞掉。
bool exit_requested();

}  // namespace tools

#endif  // TOOLS__EXITER_HPP
