#ifndef TOOLS__INSTANCE_LOCK_HPP
#define TOOLS__INSTANCE_LOCK_HPP

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace tools
{
// 单实例锁：基于 flock，进程退出（含 kill -9）时内核自动释放，不会留下假死锁。
// 构造不抛异常；用 acquired() 判断是否拿到锁，error() 取失败原因。
class InstanceLock
{
public:
  explicit InstanceLock(const std::string & path) { acquire(path); }

  ~InstanceLock()
  {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  InstanceLock(const InstanceLock &) = delete;
  InstanceLock & operator=(const InstanceLock &) = delete;

  bool acquired() const { return fd_ >= 0; }

  const std::string & error() const { return error_; }

private:
  void acquire(const std::string & path)
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd_ < 0) {
      error_ = "open lock failed: " + std::string(std::strerror(errno));
      return;
    }

    while (::flock(fd_, LOCK_EX | LOCK_NB) < 0) {
      if (errno == EINTR) continue;

      const int saved_errno = errno;
      ::close(fd_);
      fd_ = -1;
      if (saved_errno == EWOULDBLOCK || saved_errno == EAGAIN) {
        error_ = "another instance is already running";
      } else {
        error_ = "flock failed: " + std::string(std::strerror(saved_errno));
      }
      return;
    }
  }

  int fd_ = -1;
  std::string error_;
};

}  // namespace tools

#endif
