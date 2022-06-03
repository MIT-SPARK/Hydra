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
#include "hydra_utils/config_formatter.h"

#include <map>
#include <ostream>
#include <vector>

namespace config_parser {

namespace detail {

template <typename T>
void displayParam(std::ostream& out, const T& value) {
  out << value;
}

template <>
inline void displayParam<uint8_t>(std::ostream& out, const uint8_t& value) {
  out << static_cast<int>(value);
}

template <>
inline void displayParam<bool>(std::ostream& out, const bool& value) {
  out << (value ? "true" : "false");
}

// adl indirection
struct display_param_fn {
  template <typename T>
  constexpr auto operator()(std::ostream& out, const T& value) const
      -> decltype(displayParam(out, value)) {
    return displayParam(out, value);
  }
};

}  // namespace detail

namespace {

constexpr const auto& displayParam = detail::static_const<detail::display_param_fn>;

}  // namespace

// make sure vector operator is present in the right namespace
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& values) {
  out << "[";
  auto iter = values.begin();
  while (iter != values.end()) {
    out << *iter;
    ++iter;
    if (iter != values.end()) {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

template <typename K, typename V>
std::ostream& operator<<(std::ostream& out, const std::map<K, V>& values) {
  out << "{";
  auto iter = values.begin();
  while (iter != values.end()) {
    out << iter->first << ": " << iter->second;
    ++iter;
    if (iter != values.end()) {
      out << ", ";
    }
  }
  out << "}";
  return out;
}

class OstreamFormatImpl {
 public:
  explicit OstreamFormatImpl(std::ostream& out) : out_(out), prefix_("") {}

  inline OstreamFormatImpl child(const std::string& name) const {
    std::string new_prefix = "";

    const auto prev_pos = prefix_.find("-");
    if (prev_pos == std::string::npos) {
      new_prefix = "- " + name + ":";
    } else if (prev_pos == 0) {
      new_prefix = "  - " + name + ":";
    } else {
      new_prefix = "  " + prefix_.substr(0, prev_pos) + "- " + name + ":";
    }

    return OstreamFormatImpl(out_, new_prefix);
  }

  inline void pre_visit() const { out_ << prefix_; }

  inline void post_visit() const { out_ << std::endl; }

  template <typename T>
  void show(const T& value) const {
    out_ << " ";
    displayParam(out_, value);
  }

  std::string prefix() const { return prefix_; }

 private:
  OstreamFormatImpl(std::ostream& out, const std::string& prefix)
      : out_(out), prefix_(prefix) {}

  std::ostream& out_;
  std::string prefix_;
};

using OstreamFormatter = Formatter<OstreamFormatImpl>;

}  // namespace config_parser
