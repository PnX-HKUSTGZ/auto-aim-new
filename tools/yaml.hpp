#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <array>
#include <filesystem>
#include <limits.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace tools
{
inline std::filesystem::path executable_dir()
{
  std::array<char, PATH_MAX> buffer{};
  const auto length = ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
  if (length > 0) {
    return std::filesystem::path(std::string(buffer.data(), static_cast<size_t>(length)))
      .parent_path();
  }

  return std::filesystem::current_path();
}

inline std::filesystem::path resolve_path(const std::filesystem::path & path)
{
  if (path.is_absolute() && std::filesystem::exists(path)) return path;

  const auto cwd = std::filesystem::current_path();
  const auto exe_dir = executable_dir();
  const std::array<std::filesystem::path, 4> candidates = {
    path,
    cwd / path,
    exe_dir / path,
    exe_dir.parent_path() / path,
  };

  for (const auto & candidate : candidates) {
    if (std::filesystem::exists(candidate)) return candidate;
  }

  return path;
}

inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(resolve_path(path).string());
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
}

template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  logger()->error("[YAML] {} not found!", key);
  exit(1);
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP