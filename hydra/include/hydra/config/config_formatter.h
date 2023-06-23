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
#include <memory>

#include "hydra/config/config_visitor.h"

namespace config_parser {

template <typename Impl>
class Formatter {
 public:
  explicit Formatter(std::unique_ptr<Impl>&& impl)
      : impl_(std::move(impl)), root_call_(true) {}

  Formatter<Impl> operator[](const std::string& new_name) const {
    return Formatter(std::make_unique<Impl>(impl_->child(new_name)), false);
  }

  template <typename T>
  void visit(const std::string& name, T& value) const {
    auto new_parser = this->operator[](name);
    ConfigVisitor<T>::visit_config(new_parser, value);
  }

  template <typename T, typename C>
  void visit(const std::string& name, T& value, const C& converter) const {
    auto intermediate_value = converter.from(value);
    visit(name, intermediate_value);
  }

  template <typename T>
  void parse(T& value) const {
    impl_->parse(value);
  }

  inline void pre_visit() const {
    if (!root_call_) {
      impl_->pre_visit();
    }
  }

  inline void post_visit() const {
    if (!root_call_) {
      impl_->post_visit();
    }
  }

  template <typename T>
  void show(const T& value) const {
    impl_->show(value);
  }

  std::string prefix() const { return impl_->prefix(); }

 private:
  Formatter(std::unique_ptr<Impl>&& impl, bool root_call)
      : impl_(std::move(impl)), root_call_(root_call) {}

  std::unique_ptr<Impl> impl_;
  bool root_call_;
};

}  // namespace config_parser
