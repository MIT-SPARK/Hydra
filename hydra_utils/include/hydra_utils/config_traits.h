#pragma once
#include <type_traits>
#include <string>

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

template <typename T>
struct is_config : std::false_type {};

template <typename T>
struct is_parser : std::false_type {};

namespace detail {

// ODR workaround
template <class T>
constexpr T static_const{};

}  // namespace detail

}  // namespace config_parser
