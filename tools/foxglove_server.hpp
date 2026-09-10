#ifndef TOOLS__FOXGLOVE_SERVER_HPP
#define TOOLS__FOXGLOVE_SERVER_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tools
{
// JSON channels served asynchronously using the Foxglove WebSocket protocol.
// publish() replaces the latest snapshot; slow or absent clients never stall vision.
class FoxgloveServer
{
public:
  struct Channel
  {
    std::uint32_t id;
    std::string topic, schema_name, schema;
  };
  struct Message
  {
    std::uint32_t channel_id;
    std::string json;
  };
  FoxgloveServer(
    const std::string & host, std::uint16_t port, const std::vector<Channel> & channels);
  ~FoxgloveServer();
  FoxgloveServer(const FoxgloveServer &) = delete;
  FoxgloveServer & operator=(const FoxgloveServer &) = delete;

  // Atomically replace a batch, in send order (transforms before scene geometry).
  void publish(std::uint64_t timestamp_ns, std::vector<Message> messages);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tools

#endif
