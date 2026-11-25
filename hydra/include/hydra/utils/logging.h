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
#pragma once
#include <config_utilities/printing.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/ranges.h>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#ifdef SPDLOG_USE_STD_FORMAT
#error "Logging requires an fmt-based spdlog build"
#endif

#include <atomic>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "hydra/common/config_utilities.h"

namespace hydra {

struct VerbosityConfig {
  //! An unset value inherits the current run default
  std::optional<int> verbosity = std::nullopt;
  std::string log_name;
  std::string log_prefix = "[";
  std::string log_suffix = "] ";

  //! Use short snake_case names for default component log labels
  VerbosityConfig(const std::string& name = "",
                  std::optional<int> verbosity = std::nullopt);

  //! Copy with a different name while preserving verbosity and delimiters
  VerbosityConfig with_name(const std::string& name) const;

  //! Render the component label, or an empty string when the name is empty
  std::string get_log_prefix() const;

  bool enabled(int level) const { return get_verbosity() >= level; }

  int get_verbosity() const;
};

void declare_config(VerbosityConfig& config);

}  // namespace hydra

namespace hydra::logging {

std::shared_ptr<spdlog::logger> logger();
spdlog::level::level_enum parseLevel(const std::string& level);
void setLogLevel(const std::string& level);
void configureConfigLogging();
void setSinks(std::vector<spdlog::sink_ptr> sinks);
void setLogToStderr();
void setLogDirectory(const std::string& directory,
                     const std::string& stderr_level = "error");
void configurePgmoLogging(bool enabled);

// Prefer native formatting; adapt existing stream operators when needed.
template <typename T>
decltype(auto) arg(const T& value) {
  if constexpr (config::isConfig<T>()) {
    return config::toString(value);
  } else if constexpr (requires {
                         typename T::Scalar;
                         T::RowsAtCompileTime;
                         T::ColsAtCompileTime;
                       }) {
    // Eigen matrices expose begin/end even when they cannot be used as ranges.
    return fmt::streamed(value);
  } else if constexpr (fmt::is_formattable<T>::value) {
    return (value);
  } else {
    return fmt::streamed(value);
  }
}

inline std::string message() { return {}; }

template <typename... Args>
using FormatString = fmt::format_string<decltype(arg(std::declval<const Args&>()))...>;

template <typename... Args>
std::string message(FormatString<Args...> format, const Args&... args) {
  return fmt::format(format, arg(args)...);
}

template <typename... Args>
void write(const std::shared_ptr<spdlog::logger>& instance,
           spdlog::source_loc location,
           spdlog::level::level_enum level,
           FormatString<Args...> format,
           const Args&... args) {
  instance->log(location, level, format, arg(args)...);
}

[[noreturn]] void fail(spdlog::source_loc location, std::string_view message) noexcept;

// Build diagnostics inside the fatal boundary so formatting cannot escape a failure.
template <typename Diagnostic>
[[noreturn]] void failWithDiagnostic(spdlog::source_loc location,
                                     Diagnostic&& diagnostic) noexcept {
  try {
    fail(location, diagnostic());
  } catch (...) {
    fail(location, "Failed to construct fatal diagnostic");
  }
}

template <typename T>
T checkNotNull(T&& value, spdlog::source_loc location, const char* expression) {
  if (!value) {
    failWithDiagnostic(location, [&] {
      return ::fmt::format("Check failed: {} != nullptr", expression);
    });
  }

  return std::forward<T>(value);
}

}  // namespace hydra::logging

#define HYDRA_LOG_LOCATION \
  spdlog::source_loc { __FILE__, __LINE__, SPDLOG_FUNCTION }
#define HYDRA_LOG_LEVEL_INFO spdlog::level::info
#define HYDRA_LOG_LEVEL_WARNING spdlog::level::warn
#define HYDRA_LOG_LEVEL_ERROR spdlog::level::err
#define HYDRA_LOG_LEVEL_DEBUG spdlog::level::debug
#define HYDRA_LOG_LEVEL_FATAL spdlog::level::critical

#define HYDRA_LOG(LEVEL, ...)                                                          \
  do {                                                                                 \
    if constexpr (HYDRA_LOG_LEVEL_##LEVEL == spdlog::level::critical) {                \
      ::hydra::logging::failWithDiagnostic(                                            \
          HYDRA_LOG_LOCATION, [&] { return ::hydra::logging::message(__VA_ARGS__); }); \
    } else {                                                                           \
      const auto hydra_logger = ::hydra::logging::logger();                            \
      if (hydra_logger->should_log(HYDRA_LOG_LEVEL_##LEVEL)) {                         \
        ::hydra::logging::write(                                                       \
            hydra_logger, HYDRA_LOG_LOCATION, HYDRA_LOG_LEVEL_##LEVEL, __VA_ARGS__);   \
      }                                                                                \
    }                                                                                  \
  } while (false)

// Levels 1-3 cover summaries, per-update output, and normal diagnostic details.
// Reserve 4 for very detailed traces and 5 for data dumps or test diagnostics.
#define MLOG_WITH(C, V, ...)                             \
  do {                                                   \
    const auto& hydra_log_config = (C);                  \
    if (hydra_log_config.enabled(V)) {                   \
      HYDRA_LOG(INFO,                                    \
                "{}{}",                                  \
                hydra_log_config.get_log_prefix(),       \
                ::hydra::logging::message(__VA_ARGS__)); \
    }                                                    \
  } while (false)

#define MLOG(V, ...) MLOG_WITH(config, V, __VA_ARGS__)

// Severity-based module messages do not use the verbosity threshold.
#define HYDRA_MLOG_LEVEL_INFO INFO
#define HYDRA_MLOG_LEVEL_WARN WARNING
#define HYDRA_MLOG_LEVEL_ERROR ERROR

#define HYDRA_MLOG_NAMED(C, LEVEL, ...) \
  HYDRA_LOG(LEVEL, "{}{}", (C).get_log_prefix(), ::hydra::logging::message(__VA_ARGS__))
#define MLOG_NAMED_WITH(C, LEVEL, ...) \
  HYDRA_MLOG_NAMED(C, HYDRA_MLOG_LEVEL_##LEVEL, __VA_ARGS__)
#define MLOG_NAMED(LEVEL, ...) MLOG_NAMED_WITH(config, LEVEL, __VA_ARGS__)

#define HYDRA_VLOG(V, ...) MLOG_WITH(::hydra::VerbosityConfig{}, V, __VA_ARGS__)

#define HYDRA_LOG_IF(LEVEL, CONDITION, ...) \
  do {                                      \
    if (CONDITION) {                        \
      HYDRA_LOG(LEVEL, __VA_ARGS__);        \
    }                                       \
  } while (false)

// Each expansion owns a thread-safe counter, starting at one like glog.
#define HYDRA_LOG_COUNT                                       \
  [] {                                                        \
    static std::atomic<size_t> count{0};                      \
    return count.fetch_add(1, std::memory_order_relaxed) + 1; \
  }()

#define HYDRA_LOG_FIRST_N(LEVEL, N, ...)                           \
  do {                                                             \
    const auto hydra_log_count = HYDRA_LOG_COUNT;                  \
    const auto hydra_log_limit = (N);                              \
    if (hydra_log_limit > 0 &&                                     \
        hydra_log_count <= static_cast<size_t>(hydra_log_limit)) { \
      HYDRA_LOG(LEVEL, __VA_ARGS__);                               \
    }                                                              \
  } while (false)

#define HYDRA_LOG_EVERY_N(LEVEL, N, ...)                                         \
  do {                                                                           \
    const auto hydra_log_count = HYDRA_LOG_COUNT;                                \
    const auto hydra_log_period = (N);                                           \
    if (hydra_log_period > 0 && (hydra_log_count - 1) % hydra_log_period == 0) { \
      HYDRA_LOG(LEVEL, __VA_ARGS__);                                             \
    }                                                                            \
  } while (false)

#define HYDRA_CHECK(CONDITION, ...)                                   \
  do {                                                                \
    if (!(CONDITION)) {                                               \
      ::hydra::logging::failWithDiagnostic(HYDRA_LOG_LOCATION, [&] {  \
        return ::fmt::format("Check failed: {}. {}",                  \
                             #CONDITION,                              \
                             ::hydra::logging::message(__VA_ARGS__)); \
      });                                                             \
    }                                                                 \
  } while (false)

#define HYDRA_CHECK_COMPARE(A, B, OP, ...)                            \
  do {                                                                \
    const auto& hydra_check_a = (A);                                  \
    const auto& hydra_check_b = (B);                                  \
    if (!(hydra_check_a OP hydra_check_b)) {                          \
      ::hydra::logging::failWithDiagnostic(HYDRA_LOG_LOCATION, [&] {  \
        return ::fmt::format("Check failed: {} ({} vs {}). {}",       \
                             #A " " #OP " " #B,                       \
                             ::hydra::logging::arg(hydra_check_a),    \
                             ::hydra::logging::arg(hydra_check_b),    \
                             ::hydra::logging::message(__VA_ARGS__)); \
      });                                                             \
    }                                                                 \
  } while (false)

#define HYDRA_CHECK_EQ(A, B, ...) HYDRA_CHECK_COMPARE(A, B, ==, __VA_ARGS__)
#define HYDRA_CHECK_NE(A, B, ...) HYDRA_CHECK_COMPARE(A, B, !=, __VA_ARGS__)
#define HYDRA_CHECK_LT(A, B, ...) HYDRA_CHECK_COMPARE(A, B, <, __VA_ARGS__)
#define HYDRA_CHECK_LE(A, B, ...) HYDRA_CHECK_COMPARE(A, B, <=, __VA_ARGS__)
#define HYDRA_CHECK_GT(A, B, ...) HYDRA_CHECK_COMPARE(A, B, >, __VA_ARGS__)
#define HYDRA_CHECK_GE(A, B, ...) HYDRA_CHECK_COMPARE(A, B, >=, __VA_ARGS__)
#define HYDRA_CHECK_NOTNULL(V) \
  ::hydra::logging::checkNotNull((V), HYDRA_LOG_LOCATION, #V)
