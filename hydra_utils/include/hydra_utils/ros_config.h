#pragma once
#include "hydra_utils/config_visitor.h"

#include <ros/ros.h>
#include <vector>

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

template <typename T,
          typename std::enable_if<is_base_ros_param<T>::value, bool>::type = true>
void readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  std::map<std::string, T>& value) {
  if (!nh.hasParam(name)) {
    return;
  }

  value.clear();
  nh.getParam(name, value);
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
void readRosParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  int placeholder = 0;
  if (!nh.getParam(name, placeholder)) {
    return;
  }

  convertFromInt(placeholder, value);
}

template <typename T,
          typename std::enable_if<std::conjunction<std::negation<is_base_ros_param<T>>,
                                                   std::is_integral<T>>::value,
                                  bool>::type = true>
void readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  std::vector<T>& value) {
  std::vector<int> placeholders;
  if (!nh.getParam(name, placeholders)) {
    return;
  }

  for (const int placeholder : placeholders) {
    T new_value;
    convertFromInt(placeholder, new_value);
    value.push_back(new_value);
  }
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
  RosParser();

  explicit RosParser(const ros::NodeHandle& nh);

  RosParser(const ros::NodeHandle& nh, const std::string& name);

  RosParser(const RosParser& other) = default;

  RosParser& operator=(const RosParser& other) = default;

  ~RosParser() = default;

  static RosParser FromNs(const std::string& ns);

  RosParser operator[](const std::string& new_name) const;

  std::vector<std::string> children() const;

  inline std::string name() const { return name_; }

  template <typename T>
  void visit(const std::string& name, T& value) const {
    auto new_parser = this->operator[](name);
    ConfigVisitor<T>::visit_config(new_parser, value);
  }

  template <typename T, typename C>
  void visit(const std::string& name, T& value, const C& converter) const {
    auto new_parser = this->operator[](name);
    auto intermediate_value = converter.from(value);
    ConfigVisitor<T>::visit_config(new_parser, intermediate_value);
    value = converter.from(intermediate_value);
  }

  template <typename T>
  void parse(T& value) const {
    ::config_parser::readRosParam(nh_, name_, value);
  }

 private:
  ros::NodeHandle nh_;
  std::string name_;
};

}  // namespace config_parser
