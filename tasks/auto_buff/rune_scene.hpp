#ifndef AUTO_BUFF__RUNE_SCENE_HPP
#define AUTO_BUFF__RUNE_SCENE_HPP

#include <nlohmann/json.hpp>
#include <cstdint>
#include "rune_tracker.hpp"

namespace auto_buff
{
// Register both frames in Foxglove's transform tree, even with no detected rune.
nlohmann::json make_camera_transform(const CameraPose & camera, std::uint64_t timestamp_ns);

// All primitive coordinates are in world, in metres. Prediction is a motion preview,
// not the ballistic impact point calculated internally by Aimer.
nlohmann::json make_rune_scene(
  const std::optional<InactiveTargets> & observed, const std::optional<RuneTarget> & tracked,
  const CameraPose & camera, std::uint64_t timestamp_ns, double prediction_seconds = 0.2);

// Flat numeric fields can be plotted directly by Foxglove and PlotJuggler.
nlohmann::json make_rune_telemetry(
  const std::optional<InactiveTargets> & observed, const std::optional<RuneTarget> & tracked,
  double prediction_seconds = 0.2);
}  // namespace auto_buff

#endif
