#pragma once
#include "hydra_utils/config_visitor.h"

#include <memory>

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

  std::string prefix() const {
    return impl_->prefix();
  }

 private:
  Formatter(std::unique_ptr<Impl>&& impl, bool root_call)
      : impl_(std::move(impl)), root_call_(root_call) {}

  std::unique_ptr<Impl> impl_;
  bool root_call_;
};

}  // namespace config_parser
