#include "network/state_broadcaster.h"

namespace ridersense {

StateBroadcaster::StateBroadcaster(std::shared_ptr<StateServer> server,
                                   double rate_hz)
    : Service("state_broadcaster"), server_(server), rate_hz_(rate_hz),
      interval_ms_(static_cast<int>(1000.0 / rate_hz)),
      logger_(Logger::get("broadcaster")) {}

StateBroadcaster::~StateBroadcaster() { shutdown(); }

bool StateBroadcaster::init() {
  LOG_INFO(logger_, "state broadcaster initialized (rate: {} Hz)", rate_hz_);
  return true;
}

bool StateBroadcaster::start() {
  running_ = true;
  broadcast_thread_ = std::thread(&StateBroadcaster::broadcast_loop, this);
  LOG_INFO(logger_, "state broadcaster started");
  return true;
}

bool StateBroadcaster::stop() {
  LOG_INFO(logger_, "stopping state broadcaster");
  running_ = false;
  if (broadcast_thread_.joinable()) {
    broadcast_thread_.join();
  }
  return true;
}

bool StateBroadcaster::shutdown() {
  LOG_INFO(logger_, "state broadcaster shutdown");
  return true;
}

void StateBroadcaster::broadcast_loop() {
  auto last_heartbeat = std::chrono::steady_clock::now();

  while (running_) {
    std::this_thread::sleep_for(interval_ms_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_updated_ && server_->client_count() > 0) {
        StateMessage msg;
        msg.type = MessageType::STATE_UPDATE;
        msg.timestamp_ms = StateMessage::now_ms();
        msg.payload = make_state_payload(speed_, lean_, lat_, lon_, heading_);
        server_->broadcast(msg.to_json());
        state_updated_ = false;
      }
    }

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat)
            .count() >= 1) {
      send_heartbeat();
      last_heartbeat = now;
    }
  }
}

void StateBroadcaster::send_heartbeat() {
  if (server_->client_count() == 0) {
    return;
  }

  StateMessage msg;
  msg.type = MessageType::HEARTBEAT;
  msg.timestamp_ms = StateMessage::now_ms();
  server_->broadcast(msg.to_json());
}

void StateBroadcaster::on_fusion(std::shared_ptr<FusionFrame> fusion,
                                 std::shared_ptr<GpsFrame> gps) {
  if (!fusion) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  speed_ = fusion->speed;
  lean_ = fusion->lean_angle;

  if (gps) {
    lat_ = gps->latitude;
    lon_ = gps->longitude;
    heading_ = gps->heading;
  }

  state_updated_ = true;
}

void StateBroadcaster::on_event(const Event &event) {
  if (server_->client_count() == 0) {
    return;
  }

  StateMessage msg;
  msg.type = MessageType::EVENT;
  msg.timestamp_ms = StateMessage::now_ms();
  msg.payload = make_event_payload(Event::type_to_string(event.type),
                                   Event::severity_to_string(event.severity),
                                   event.value);
  server_->broadcast(msg.to_json());
}

void StateBroadcaster::on_metrics(const RidingMetrics &metrics) {
  if (server_->client_count() == 0) {
    return;
  }

  StateMessage msg;
  msg.type = MessageType::METRICS;
  msg.timestamp_ms = StateMessage::now_ms();
  msg.payload = make_metrics_payload(metrics.max_speed, metrics.avg_speed,
                                     metrics.hard_brake_count,
                                     metrics.excessive_lean_count);
  server_->broadcast(msg.to_json());
}

} // namespace ridersense
