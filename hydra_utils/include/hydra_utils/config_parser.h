#pragma once
#include "hydra_utils/config_visitor.h"

#include <memory>
#include <vector>

namespace config_parser {

template <typename Impl>
class Parser {
 public:
  Parser(std::unique_ptr<Impl>&& impl) : impl_(std::move(impl)) {}

  ~Parser() = default;

  Parser(const Parser& other) = delete;

  Parser(Parser&& other) = delete;

  Parser<Impl> operator[](const std::string& new_name) const {
    return Parser(std::make_unique<Impl>(impl_->child(new_name)));
  }

  template <typename T>
  void visit(const std::string& name, T& value) const {
    auto new_parser = this->operator[](name);
    ConfigVisitor<T>::visit_config(new_parser, value);
  }

  template <typename T, typename C>
  void visit(const std::string& name, T& value, const C& converter) const {
    auto intermediate_value = converter.from(value);
    this->visit(name, intermediate_value);
    value = converter.to(intermediate_value);
  }

  std::vector<std::string> children() const { return impl_->children(); }

  template <typename T>
  void parse(T& value) const {
    impl_->parse(value);
  }

 private:
  std::unique_ptr<Impl> impl_;
};

template <typename T>
struct is_parser<Parser<T>> : std::true_type {};

}  // namespace config_parser
