#pragma once

#include <spdlog/sinks/base_sink.h>

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace hydra {

class RosSink : public spdlog::sinks::base_sink<std::mutex> {
 public:
  explicit RosSink(rclcpp::Logger logger) : logger_(logger.get_child("spdlog")) {
    // Let spdlog filter forwarded messages without changing the node's threshold.
    logger_.set_level(rclcpp::Logger::Level::Debug);
  }

 protected:
  void sink_it_(const spdlog::details::log_msg& entry) override {
    const auto payload = std::string(entry.payload.data(), entry.payload.size());
    const auto message =
        entry.source.empty()
            ? payload
            : fmt::format(
                  "[{}:{}] {}", entry.source.filename, entry.source.line, payload);
    switch (entry.level) {
      case spdlog::level::warn:
        RCLCPP_WARN(logger_, "%s", message.c_str());
        break;
      case spdlog::level::err:
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        break;
      case spdlog::level::critical:
        RCLCPP_FATAL(logger_, "%s", message.c_str());
        break;
      case spdlog::level::info:
        RCLCPP_INFO(logger_, "%s", message.c_str());
        break;
      default:
        RCLCPP_DEBUG(logger_, "%s", message.c_str());
        break;
    }
  }

  void flush_() override {}

 private:
  rclcpp::Logger logger_;
};

}  // namespace hydra
