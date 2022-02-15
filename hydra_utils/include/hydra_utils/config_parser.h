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
template <typename Visitor,
          typename T,
          typename std::enable_if<std::negation<is_parser<Visitor>>::value,
                                  bool>::type = true>
void visit_config(const Visitor& v, const T& val) {
  v.visit(val);
  v.post_visit();
}

// default for primitive types: makes sure that the visitor bottoms-out on leaves
template <typename Visitor,
          typename T,
          typename std::enable_if<is_parser<Visitor>::value, bool>::type = true>
void visit_config(const Visitor& v, const T& val) {
  // revert to non-const for leaves (so that the visitor can actually modify the
  // configs)
  v.visit(const_cast<T&>(val));
}

// adl indirection
struct visit_config_fn {
  template <typename Visitor,
            typename ValueType,
            typename std::enable_if<is_parser<Visitor>::value, bool>::type = true>
  constexpr auto operator()(const Visitor& v, ValueType& val) const
      -> decltype(visit_config(v, static_cast<const ValueType&>(val))) {
    return visit_config(v, static_cast<const ValueType&>(val));
  }

  template <typename Visitor,
            typename ValueType,
            typename std::enable_if<std::negation<is_parser<Visitor>>::value,
                                    bool>::type = true>
  constexpr auto operator()(const Visitor& v, const ValueType& val) const
      -> decltype(visit_config(v, val)) {
    v.pre_visit();
    if (is_config<ValueType>()) {
      v.post_visit();
    }
    visit_config(v, val);
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
  static auto visit_config(const Visitor& v, const ValueType& val)
      -> decltype(::config_parser::visit_config(v, val), void()) {
    ::config_parser::visit_config(v, val);
  }
};

}  // namespace config_parser
