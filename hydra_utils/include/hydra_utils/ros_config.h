#pragma once
#include <ros/ros.h>

#include <type_traits>

namespace kimera {

template <typename T>
struct is_base_ros_param : std::false_type {};

template <>
struct is_base_ros_param<std::string> : std::true_type {};

template <>
struct is_base_ros_param<double> : std::true_type {};

template <>
struct is_base_ros_param<float> : std::true_type {};

template <>
struct is_base_ros_param<int> : std::true_type {};

template <>
struct is_base_ros_param<bool> : std::true_type {};

template <typename T, std::enable_if_t<is_base_ros_param<T>::value, bool> = true>
bool parseParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  return nh.getParam(name, value);
}

template <typename T, std::enable_if_t<is_base_ros_param<T>::value, bool> = true>
bool parseParam(const ros::NodeHandle& nh,
                const std::string& name,
                std::vector<T>& value) {
  return nh.getParam(name, value);
}

template <typename T,
          typename Bounds,
          std::enable_if_t<std::is_integral<T>::value, bool> = true>
bool parseParam(const ros::NodeHandle& nh,
                const std::string& name,
                T& value,
                const Bounds lo_value,
                const Bounds hi_value) {
  int placeholder = 0;  // putting default value into int is overflow prone
  bool had_param = nh.getParam(name, placeholder);
  if (!had_param) {
    return false;
  }

  if (sizeof(T) < sizeof(int)) {
    // avoid overflow on uint16_t and smaller
    int old_placeholder = placeholder;
    placeholder =
        std::clamp(placeholder, static_cast<int>(lo_value), static_cast<int>(hi_value));
    if (placeholder != old_placeholder) {
      ROS_WARN_STREAM("Parameter "
                      << nh.resolveName(name) << " had a value " << old_placeholder
                      << " which was outside the bounds of [" << lo_value << ", "
                      << hi_value << "]");
    }
  }

  value = static_cast<T>(placeholder);

  if (sizeof(T) >= sizeof(int)) {
    // avoid clamping issues with values larger than int
    T old_value = value;
    value = std::clamp(value, static_cast<T>(lo_value), static_cast<T>(hi_value));
    if (value != old_value) {
      ROS_WARN_STREAM("Parameter " << nh.resolveName(name) << " had a value "
                                   << old_value << " which was outside the bounds of ["
                                   << lo_value << ", " << hi_value << "]");
    }
  }
  return had_param;
}

template <typename T,
          std::enable_if_t<!is_base_ros_param<T>::value && std::is_integral<T>::value,
                           bool> = true>
bool parseParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  return parseParam(
      nh, name, value, std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
}

template <typename T>
void showParam(std::ostream& out, const T& value) {
  out << value;
}

template <>
void showParam(std::ostream& out, const uint8_t& value) {
  out << static_cast<int>(value);
}

template <>
void showParam(std::ostream& out, const bool& value) {
  out << (value ? "yes" : "no");
}

struct RosParser {

  explicit RosParser(const ros::NodeHandle& nh) : nh_(nh) {}

  template <typename T>
  void call(const std::string& name, T& value) const {
    parseParam(nh_, name, value);
  }

  ros::NodeHandle nh_;
};

} // namespace kimera
