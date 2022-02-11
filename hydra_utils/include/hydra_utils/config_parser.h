#pragma once
#include "hydra_utils/config_traits.h"
#include <string>
#include <utility>

// argument-dependent-lookup for arbitrary config structures. See the following:
// - http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4381.html
// - https:://github.com/nlohmann/json/blob/develop/include/nlohmann/adl_serializer.hpp

namespace config_parser {

namespace detail {

// default for primitive types: makes sure that the visitor bottoms-out on leaves
template <typename Visitor, typename T>
void visit_config(Visitor&& v, T& val) {
  std::forward<Visitor>(v).visit(val);
}

// adl indirection
struct visit_config_fn {
  template <typename Visitor, typename ValueType>
  constexpr auto operator()(Visitor&& v, ValueType& val) const
      -> decltype(visit_config(std::forward<Visitor>(v), val)) {
    return visit_config(v, val);
  }
};

}  // namespace detail

namespace {

constexpr const auto& visit_config = detail::static_const<detail::visit_config_fn>;

}  // namespace

// TODO(nathan) this might need more work
template <typename T = void, typename SFINAE = void>
struct ConfigVisitorSpecializer;

template <typename, typename>
struct ConfigVisitorSpecializer {
  /*
   * @brief apply visitor to any value type
   */
  template <typename Visitor, typename ValueType>
  static auto visit_config(Visitor&& v, ValueType& val)
      -> decltype(::config_parser::visit_config(std::forward<Visitor>(v), val),
                  void()) {
    ::config_parser::visit_config(std::forward<Visitor>(v), val);
  }
};

}  // namespace config_parser
