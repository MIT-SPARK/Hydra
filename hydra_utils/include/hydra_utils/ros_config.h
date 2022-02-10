#pragma once
#include <ros/ros.h>

#include <type_traits>

namespace config_parser {

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

class RosParser {
 public:
  RosParser(const ros::NodeHandle& nh, const std::string& name)
      : nh_(nh), name_(name) {}

  explicit RosParser(const ros::NodeHandle& nh) : RosParser(nh, "") {}

  static RosParser FromNs(const std::string& ns) { return RosParser(ros::NodeHandle(ns)); }

  RosParser operator[](const std::string& new_name) const {
    // push name onto nodehandle namespace if name isn't empty
    ros::NodeHandle new_nh = (name_ != "") ? nh_ : ros::NodeHandle(nh_, name_);
    return RosParser(new_nh, new_name);
  }

  template <typename T>
  void visit(T& value) const;

  template <typename T, std::enable_if_t<is_base_ros_param<T>::value, bool> = true>
  void visit(T& value) {
    nh_.getParam(name_, value);
  }

  // TODO(nathan) map
  template <typename T, std::enable_if_t<is_base_ros_param<T>::value, bool> = true>
  void visit(std::vector<T>& value) {
    nh_.getParam(name_, value);
  }

  template <typename T,
            std::enable_if_t<std::conjunction<std::negation<is_base_ros_param<T>>,
                                              std::is_integral<T>>::value,
                             bool> = true>
  void visit(T& value) {
    int placeholder = 0;
    if (!nh_.getParam(name_, placeholder)) {
      return;
    }

    if (sizeof(T) < sizeof(int)) {
      // avoid overflow on uint16_t and smaller
      constexpr const int lo_value = static_cast<int>(std::numeric_limits<T>::min());
      constexpr const int hi_value = static_cast<int>(std::numeric_limits<T>::max());
      int old_placeholder = placeholder;
      placeholder = std::clamp(placeholder, lo_value, hi_value);
      if (placeholder != old_placeholder) {
        ROS_WARN_STREAM("Parameter "
                        << nh_.resolveName(name_) << " had a value " << old_placeholder
                        << " which was outside the bounds of [" << lo_value << ", "
                        << hi_value << "]");
      }
    }

    value = static_cast<T>(placeholder);
  }

 private:
  ros::NodeHandle nh_;
  std::string name_;
};

}  // namespace config_parser
