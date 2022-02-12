#pragma once
#include <type_traits>
#include <string>

namespace config_parser {

template <typename T>
struct is_base_ros_param : public std::false_type {};

template <>
struct is_base_ros_param<std::string> : public std::true_type {};

template <>
struct is_base_ros_param<double> : public std::true_type {};

template <>
struct is_base_ros_param<float> : public std::true_type {};

template <>
struct is_base_ros_param<int> : public std::true_type {};

template <>
struct is_base_ros_param<bool> : public std::true_type {};

template <typename T>
struct is_config : public std::false_type {};

template <typename T>
struct is_parser : public std::false_type {};

namespace detail {

// ODR workaround
template <class T>
constexpr T static_const{};

}  // namespace detail

}  // namespace config_parser
