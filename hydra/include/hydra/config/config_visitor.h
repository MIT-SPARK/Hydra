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
#include <string>
#include <utility>

#include "hydra/config/config_traits.h"

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
  visitor.show(value);
  visitor.post_visit();
}

// default for primitive types: makes sure that the visitor bottoms-out on leaves
template <typename V,
          typename T,
          typename std::enable_if<is_parser<V>::value, bool>::type = true>
void visit_config(const V& visitor, T& value) {
  visitor.parse(value);
}

// adl indirection
struct visit_config_fn {
  template <typename V, typename T>
  constexpr auto operator()(const V& visitor, T& value) const
      -> decltype(visit_config(visitor, value)) {
    return visit_config(visitor, value);
  }
};

}  // namespace detail

namespace {

constexpr const auto& visit_config = detail::static_const<detail::visit_config_fn>;

}  // namespace

template <typename ValueType = void, typename SFINAE = void>
struct ConfigVisitor {
  template <typename V,
            typename T = ValueType,
            typename std::enable_if<is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& visitor, T& value)
      -> decltype(::config_parser::visit_config(visitor, value), void()) {
    return ::config_parser::visit_config(visitor, value);
  }

  template <typename V,
            typename T = ValueType,
            typename std::enable_if<!is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& visitor, T& value)
      -> decltype(::config_parser::visit_config(visitor, value), void()) {
    visitor.pre_visit();

    if (is_config<T>()) {
      visitor.post_visit();
    }

    return ::config_parser::visit_config(visitor, value);
  }
};

}  // namespace config_parser
