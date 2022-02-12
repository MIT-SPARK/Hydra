#pragma once
#include <ros/ros.h>
#include "hydra_utils/config_traits.h"

namespace config_parser {

namespace detail {

template <typename T,
          typename std::enable_if<is_base_ros_param<T>::value, bool>::type = true>
void readRosParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  nh.getParam(name, value);
}

template <typename T,
          typename std::enable_if<is_base_ros_param<T>::value, bool>::type = true>
void readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  std::vector<T>& value) {
  nh.getParam(name, value);
}

template <typename K,
          typename V,
          typename std::enable_if<
              std::conjunction<is_base_ros_param<K>, is_base_ros_param<V>>::value,
              bool>::type = true>
void readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  std::map<K, V>& value) {
  nh.getParam(name, value);
}

template <typename T,
          typename std::enable_if<std::conjunction<std::negation<is_base_ros_param<T>>,
                                                   std::is_integral<T>>::value,
                                  bool>::type = true>
void readRosParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  int placeholder = 0;
  if (!nh.getParam(name, placeholder)) {
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
                      << nh.resolveName(name) << " had a value " << old_placeholder
                      << " which was outside the bounds of [" << lo_value << ", "
                      << hi_value << "]");
    }
  }

  value = static_cast<T>(placeholder);
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

class RosParser {
 public:
  RosParser(const ros::NodeHandle& nh, const std::string& name);

  explicit RosParser(const ros::NodeHandle& nh);

  ~RosParser() = default;

  static RosParser FromNs(const std::string& ns);

  RosParser operator[](const std::string& new_name) const;

  template <typename T>
  void visit(T& value) {
    ::config_parser::readRosParam(nh_, name_, value);
  }

 private:
  ros::NodeHandle nh_;
  std::string name_;
};

}  // namespace config_parser
