/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/utils/logging.h"

#include <config_utilities/config.h>
#include <config_utilities/logging/log_to_spdlog.h>
#include <config_utilities/logging/set_logger.h>
#include <kimera_pgmo/utils/logging.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

#include "hydra/common/run_settings.h"

namespace hydra {

void declare_config(VerbosityConfig& config) {
  config::field(config.verbosity, "verbosity");
  config::field(config.show_log_prefix, "show_log_prefix");
  config::field(config.log_name, "log_name");
  config::field(config.log_prefix, "log_prefix");
  config::field(config.log_suffix, "log_suffix");
}

VerbosityConfig::VerbosityConfig(const std::string& name, std::optional<int> verbosity)
    : verbosity(verbosity), log_name(name) {}

VerbosityConfig VerbosityConfig::with_name(const std::string& name) const {
  auto new_config = *this;
  new_config.log_name = name;
  return new_config;
}

std::string VerbosityConfig::get_log_prefix() const {
  return !show_log_prefix || log_name.empty() ? "" : log_prefix + log_name + log_suffix;
}

int VerbosityConfig::get_verbosity() const {
  return verbosity.value_or(RunSettings::instance().default_verbosity);
}

}  // namespace hydra

namespace hydra::logging {
namespace {

constexpr auto kLogPattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v";

spdlog::sink_ptr& runFileSink() {
  static spdlog::sink_ptr sink;
  return sink;
}

std::atomic<bool>& pgmoEnabled() {
  static std::atomic<bool> enabled{true};
  return enabled;
}

}  // namespace

std::shared_ptr<spdlog::logger> logger() {
  static const auto instance = [] {
    if (auto existing = spdlog::get("hydra")) {
      return existing;
    }

    auto result = spdlog::stderr_color_mt("hydra");
    result->set_pattern(kLogPattern);
    return result;
  }();
  return instance;
}

spdlog::level::level_enum parseLevel(const std::string& level) {
  if (level == "trace" || level == "debug" || level == "info" || level == "warning" ||
      level == "warn" || level == "error" || level == "critical" || level == "off") {
    return spdlog::level::from_str(level);
  }

  throw std::invalid_argument("Invalid log level: " + level);
}

void configureConfigLogging() {
  config::setCustomLogger(std::make_unique<config::SpdlogLogger>(logger()));
}

void setLogLevel(const std::string& level) {
  const auto parsed = parseLevel(level);
  configureConfigLogging();
  logger()->set_level(parsed);
  const auto pgmo = kimera_pgmo::getLogger();
  if (pgmoEnabled().load()) {
    pgmo->set_level(parsed);
  }
}

void setSinks(std::vector<spdlog::sink_ptr> sinks) {
  if (sinks.empty()) {
    throw std::invalid_argument("At least one logging sink is required");
  }

  logger()->flush();
  logger()->sinks() = std::move(sinks);
  kimera_pgmo::getLogger()->sinks() = logger()->sinks();
  configureConfigLogging();
}

void setLogToStderr() {
  setSinks({std::make_shared<spdlog::sinks::stderr_color_sink_mt>()});
  logger()->set_pattern(kLogPattern);
}

void setLogDirectory(const std::string& directory, const std::string& stderr_level) {
  const auto threshold = parseLevel(stderr_level);
  if (directory.empty()) {
    throw std::invalid_argument("Log directory must not be empty");
  }

  std::filesystem::create_directories(directory);
  const auto file = std::filesystem::path(directory) / "hydra.log";
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file.string());
  auto console = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  console->set_level(threshold);
  setSinks({file_sink, console});
  logger()->set_pattern(kLogPattern);
}

void setRunLogDirectory(const std::optional<std::string>& directory) {
  if (!directory && !runFileSink()) {
    return;
  }

  spdlog::sink_ptr next;
  if (directory) {
    if (!std::filesystem::is_directory(*directory)) {
      throw std::invalid_argument("Run log directory does not exist: " + *directory);
    }

    const auto file = std::filesystem::path(*directory) / "hydra.log";
    next = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file.string());
    next->set_pattern(kLogPattern);
  }

  auto sinks = logger()->sinks();
  sinks.erase(std::remove(sinks.begin(), sinks.end(), runFileSink()), sinks.end());
  if (next) {
    sinks.push_back(next);
  }

  if (sinks.empty()) {
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());
    sinks.back()->set_pattern(kLogPattern);
  }

  setSinks(std::move(sinks));
  runFileSink() = std::move(next);
}

void configurePgmoLogging(bool enabled) {
  pgmoEnabled().store(enabled);
  const auto pgmo = kimera_pgmo::getLogger();
  pgmo->flush();
  pgmo->sinks() = logger()->sinks();
  pgmo->set_level(enabled ? logger()->level() : spdlog::level::off);
}

[[noreturn]] void fail(spdlog::source_loc location, std::string_view message) noexcept {
  try {
    const auto instance = logger();
    instance->set_level(spdlog::level::critical);
    instance->log(location, spdlog::level::critical, "{}", message);
    instance->flush();
  } catch (...) {
    // Preserve the failure location even if logger initialization or a sink fails.
    std::fprintf(
        stderr, "[%s:%d] Fatal logging failed\n", location.filename, location.line);
  }

  std::abort();
}

}  // namespace hydra::logging
