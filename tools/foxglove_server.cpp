#include "foxglove_server.hpp"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <deque>
#include <exception>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace tools
{
namespace
{
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using Json = nlohmann::json;

void append_le(std::string & buffer, std::uint64_t value, unsigned bytes)
{
  for (unsigned i = 0; i < bytes; ++i) buffer.push_back(static_cast<char>(value >> (8 * i)));
}

struct Session : std::enable_shared_from_this<Session>
{
  websocket::stream<beast::tcp_stream> ws;
  beast::flat_buffer input;
  struct Message { bool binary; std::string data; };
  std::deque<Message> output;
  struct Subscription { std::uint32_t channel_id; std::uint64_t sent_sequence; };
  std::map<std::uint32_t, Subscription> subscriptions;
  std::set<std::uint32_t> channel_ids;
  bool ready = false;
  bool writing = false;

  Session(tcp::socket socket, const std::set<std::uint32_t> & channels)
  : ws(std::move(socket)), channel_ids(channels) {}

  void start(const std::string & advertisement)
  {
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws.set_option(websocket::stream_base::decorator([](websocket::response_type & response) {
      response.set(beast::http::field::sec_websocket_protocol, "foxglove.websocket.v1");
    }));
    ws.read_message_max(64 * 1024);
    ws.async_accept([self = shared_from_this(), advertisement](beast::error_code ec) {
      if (ec) return;
      self->ready = true;
      self->enqueue(false, Json({{"op", "serverInfo"}, {"name", "auto_buff_debug_mpc"},
                                {"capabilities", Json::array()}}).dump());
      self->enqueue(false, advertisement);
      self->read();
    });
  }

  void stop()
  {
    ready = false;
    beast::error_code ignored;
    beast::get_lowest_layer(ws).socket().close(ignored);
  }

  void read()
  {
    ws.async_read(input, [self = shared_from_this()](beast::error_code ec, std::size_t) {
      if (ec) { self->stop(); return; }
      const auto message = Json::parse(beast::buffers_to_string(self->input.data()), nullptr, false);
      self->input.consume(self->input.size());
      if (self->ws.got_text() && message.is_object()) {
        try {
          const auto op = message.value("op", std::string());
          if (op == "subscribe") {
            for (const auto & sub : message.at("subscriptions")) {
              const auto channel_id = sub.at("channelId").get<std::uint32_t>();
              if (self->channel_ids.count(channel_id) && self->subscriptions.size() < 16)
                self->subscriptions.emplace(
                  sub.at("id").get<std::uint32_t>(), Subscription{channel_id, 0});
            }
          } else if (op == "unsubscribe") {
            for (const auto & id : message.at("subscriptionIds"))
              self->subscriptions.erase(id.get<std::uint32_t>());
          }
        } catch (const Json::exception &) {
          // Ignore malformed control messages and keep serving valid clients.
        }
      }
      self->read();
    });
  }

  void enqueue(bool binary, std::string data)
  {
    if (!ready) return;
    output.push_back({binary, std::move(data)});
    write();
  }

  void write()
  {
    if (!ready || writing || output.empty()) return;
    writing = true;
    try {
      ws.binary(output.front().binary);
      ws.async_write(
        asio::buffer(output.front().data),
        [self = shared_from_this()](beast::error_code ec, std::size_t) {
          self->writing = false;
          if (ec) {
            self->stop();
            return;
          }
          self->output.pop_front();
          self->write();
        });
    } catch (const std::exception &) {
      writing = false;
      stop();
    }
  }

  void send_snapshot(
    std::uint64_t sequence, std::uint64_t timestamp,
    const std::vector<FoxgloveServer::Message> & messages)
  {
    if (!ready) return;
    for (const auto & message : messages) {
      for (auto & [id, subscription] : subscriptions) {
        if (subscription.channel_id != message.channel_id ||
            subscription.sent_sequence == sequence || output.size() >= 4) continue;
        std::string packet(1, '\x01');
        append_le(packet, id, 4);
        append_le(packet, timestamp, 8);
        packet += message.json;
        enqueue(true, std::move(packet));
        subscription.sent_sequence = sequence;
      }
    }
  }
};
}  // namespace

struct FoxgloveServer::Impl
{
  asio::io_context context;
  tcp::acceptor acceptor;
  asio::steady_timer timer{context};
  std::thread thread;
  std::vector<std::weak_ptr<Session>> sessions;
  std::string advertisement;
  std::set<std::uint32_t> channel_ids;
  std::mutex mutex;
  std::vector<FoxgloveServer::Message> latest;
  std::uint64_t timestamp = 0;
  std::uint64_t sequence = 0;

  Impl(const std::string & host, std::uint16_t port,
       const std::vector<FoxgloveServer::Channel> & channels)
  : acceptor(context)
  {
    const tcp::endpoint endpoint(asio::ip::make_address(host), port);
    acceptor.open(endpoint.protocol());
    acceptor.set_option(asio::socket_base::reuse_address(true));
    acceptor.bind(endpoint);
    acceptor.listen();
    Json advertised_channels = Json::array();
    for (const auto & channel : channels) {
      channel_ids.insert(channel.id);
      advertised_channels.push_back({
        {"id", channel.id}, {"topic", channel.topic}, {"encoding", "json"},
        {"schemaName", channel.schema_name}, {"schemaEncoding", "jsonschema"},
        {"schema", channel.schema}});
    }
    advertisement = Json({{"op", "advertise"}, {"channels", advertised_channels}}).dump();
    accept();
    tick();
    thread = std::thread([this] {
      // A client can disappear between readiness checks and an async operation.
      // Never let a synchronous Beast exception terminate the vision process.
      while (!context.stopped()) {
        try {
          context.run();
          break;
        } catch (const std::exception &) {
        }
      }
    });
  }

  ~Impl()
  {
    context.stop();
    if (thread.joinable()) thread.join();
    for (auto & weak : sessions) if (auto session = weak.lock()) session->stop();
  }

  void accept()
  {
    acceptor.async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (!ec) {
        auto session = std::make_shared<Session>(std::move(socket), channel_ids);
        sessions.emplace_back(session);
        session->start(advertisement);
      }
      if (acceptor.is_open()) accept();
    });
  }

  void tick()
  {
    timer.expires_after(std::chrono::milliseconds(33));
    timer.async_wait([this](beast::error_code ec) {
      if (ec) return;
      std::vector<FoxgloveServer::Message> messages;
      std::uint64_t stamp, seq;
      {
        std::lock_guard<std::mutex> lock(mutex);
        messages = latest;
        stamp = timestamp;
        seq = sequence;
      }
      for (auto it = sessions.begin(); it != sessions.end();) {
        if (auto session = it->lock()) {
          if (seq != 0) session->send_snapshot(seq, stamp, messages);
          ++it;
        } else {
          it = sessions.erase(it);
        }
      }
      tick();
    });
  }
};

FoxgloveServer::FoxgloveServer(
  const std::string & host, std::uint16_t port, const std::vector<Channel> & channels)
: impl_(std::make_unique<Impl>(host, port, channels)) {}

FoxgloveServer::~FoxgloveServer() = default;

void FoxgloveServer::publish(std::uint64_t timestamp_ns, std::vector<Message> messages)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->timestamp = timestamp_ns;
  impl_->latest = std::move(messages);
  ++impl_->sequence;
}
}  // namespace tools
