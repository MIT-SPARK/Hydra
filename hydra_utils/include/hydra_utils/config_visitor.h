#pragma once
#include "hydra_utils/config_traits.h"

#include <string>
#include <utility>

// argument-dependent-lookup for arbitrary config structures. See the following:
// - http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4381.html
// - https:://github.com/nlohmann/json/blob/develop/include/nlohmann/adl_serializer.hpp

namespace config_parser {

namespace detail {

// specialize primitive types so that non-parsers get a little more information
template <typename V,
          typename T,
          typename std::enable_if<!is_parser<V>::value, bool>::type = true>
void visit_config(const V& visitor, T& value) {
  visitor.visit(value);
  visitor.post_visit();
}

// default for primitive types: makes sure that the visitor bottoms-out on leaves
template <typename V,
          typename T,
          typename std::enable_if<is_parser<V>::value, bool>::type = true>
void visit_config(const V& visitor, T& value) {
  visitor.visit(value);
}

// adl indirection
struct visit_config_fn {
  template <typename V,
            typename T,
            typename std::enable_if<is_parser<V>::value, bool>::type = true>
  constexpr auto operator()(const V& visitor, T& value) const
      -> decltype(visit_config(visitor, value)) {
    return visit_config(visitor, value);
  }

  template <typename V,
            typename T,
            typename C,
            typename std::enable_if<is_parser<V>::value, bool>::type = true>
  constexpr void operator()(const V& visitor, T& value, const C& converter) const {
    auto intermediate_value = converter.from(value);
    this->operator()(visitor, intermediate_value);
    value = converter.from(intermediate_value);
  }

  template <typename V,
            typename T,
            typename std::enable_if<!is_parser<V>::value, bool>::type = true>
  constexpr auto operator()(const V& visitor, T& value) const
      -> decltype(visit_config(visitor, value)) {
    visitor.pre_visit();
    if (is_config<T>()) {
      visitor.post_visit();
    }
    visit_config(visitor, value);
  }

  template <typename V,
            typename T,
            typename C,
            typename std::enable_if<!is_parser<V>::value, bool>::type = true>
  constexpr void operator()(const V& visitor, T& value, const C& converter) const {
    auto intermediate_value = converter.from(value);
    this->operator()(visitor, intermediate_value);
  }

};

}  // namespace detail

namespace {

constexpr const auto& visit_config = detail::static_const<detail::visit_config_fn>;

}  // namespace

}  // namespace config_parser
