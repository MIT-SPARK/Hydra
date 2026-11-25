#pragma once

#include <rcutils/logging.h>
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
    int severity = RCUTILS_LOG_SEVERITY_DEBUG;
    switch (entry.level) {
      case spdlog::level::warn:
        severity = RCUTILS_LOG_SEVERITY_WARN;
        break;
      case spdlog::level::err:
        severity = RCUTILS_LOG_SEVERITY_ERROR;
        break;
      case spdlog::level::critical:
        severity = RCUTILS_LOG_SEVERITY_FATAL;
        break;
      case spdlog::level::info:
        severity = RCUTILS_LOG_SEVERITY_INFO;
        break;
      case spdlog::level::off:
        return;
      default:
        break;
    }

    const rcutils_log_location_t location{entry.source.funcname,
                                          entry.source.filename,
                                          static_cast<size_t>(entry.source.line)};
    const auto payload = std::string(entry.payload.data(), entry.payload.size());
    rcutils_log(entry.source.empty() ? nullptr : &location,
                severity,
                logger_.get_name(),
                "%s",
                payload.c_str());
  }

  void flush_() override {}

 private:
  rclcpp::Logger logger_;
};

}  // namespace hydra
