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
#include "hydra_utils/config_parser.h"

#include <ros/ros.h>
#include <vector>

namespace config_parser {

namespace detail {

template <typename T,
          typename std::enable_if<is_base_ros_param<T>::value, bool>::type = true>
bool readRosParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  return nh.getParam(name, value);
}

template <typename T,
          typename std::enable_if<is_base_ros_param<T>::value, bool>::type = true>
bool readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  std::vector<T>& value) {
  return nh.getParam(name, value);
}

template <typename T,
          typename std::enable_if<is_base_ros_param<T>::value, bool>::type = true>
bool readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  std::map<std::string, T>& value) {
  if (!nh.hasParam(name)) {
    return false;
  }

  value.clear();
  nh.getParam(name, value);
  return true;
}

template <typename T>
void convertFromInt(int placeholder, T& value) {
  if (sizeof(T) < sizeof(int)) {
    // avoid overflow on uint16_t and smaller
    constexpr const int lo_value = static_cast<int>(std::numeric_limits<T>::min());
    constexpr const int hi_value = static_cast<int>(std::numeric_limits<T>::max());
    placeholder = std::clamp(placeholder, lo_value, hi_value);
    // TODO(nathan) think about warning
  }

  value = static_cast<T>(placeholder);
}

template <typename T,
          typename std::enable_if<std::conjunction<std::negation<is_base_ros_param<T>>,
                                                   std::is_integral<T>>::value,
                                  bool>::type = true>
bool readRosParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  int placeholder = 0;
  if (!nh.getParam(name, placeholder)) {
    return false;
  }

  convertFromInt(placeholder, value);
  return true;
}

template <typename T,
          typename std::enable_if<std::conjunction<std::negation<is_base_ros_param<T>>,
                                                   std::is_integral<T>>::value,
                                  bool>::type = true>
bool readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  std::vector<T>& value) {
  std::vector<int> placeholders;
  if (!nh.getParam(name, placeholders)) {
    return false;
  }

  for (const int placeholder : placeholders) {
    T new_value;
    convertFromInt(placeholder, new_value);
    value.push_back(new_value);
  }

  return true;
}

// adl indirection
struct read_ros_param_fn {
  template <typename T>
  constexpr auto operator()(const ros::NodeHandle& nh,
                            const std::string& name,
                            T& val) const -> decltype(readRosParam(nh, name, val)) {
    return readRosParam(nh, name, val);
  }
};

}  // namespace detail

namespace {

constexpr const auto& readRosParam = detail::static_const<detail::read_ros_param_fn>;

}  // namespace

class RosParserImpl {
 public:
  RosParserImpl();

  explicit RosParserImpl(const ros::NodeHandle& nh);

  RosParserImpl(const ros::NodeHandle& nh, const std::string& name);

  RosParserImpl(const RosParserImpl& other) = default;

  RosParserImpl& operator=(const RosParserImpl& other) = default;

  ~RosParserImpl() = default;

  RosParserImpl child(const std::string& new_name) const;

  std::vector<std::string> children() const;

  inline std::string name() const { return nh_.resolveName(name_); }

  template <typename T>
  bool parse(T& value) const {
    return ::config_parser::readRosParam(nh_, name_, value);
  }

 private:
  ros::NodeHandle nh_;
  std::string name_;
};

using RosParser = Parser<RosParserImpl>;

}  // namespace config_parser
