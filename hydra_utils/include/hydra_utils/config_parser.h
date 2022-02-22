#pragma once
#include "hydra_utils/config_visitor.h"
#include "hydra_utils/ostream_formatter.h"

#include <memory>
#include <sstream>
#include <vector>

namespace config_parser {

struct Logger {
  using Ptr = std::shared_ptr<Logger>;

  virtual void log_missing(const std::string& message) const = 0;
};

template <typename Impl>
class Parser {
 public:
  Parser(std::unique_ptr<Impl>&& impl, Logger::Ptr logger = nullptr)
      : impl_(std::move(impl)), logger_(logger) {}

  ~Parser() = default;

  Parser(const Parser& other) = delete;

  Parser(Parser&& other) = delete;

  Parser<Impl> operator[](const std::string& new_name) const {
    return Parser(std::make_unique<Impl>(impl_->child(new_name)), logger_);
  }

  void setLogger(const Logger::Ptr& logger) { logger_ = logger; }

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
    const bool found = impl_->parse(value);
    if (logger_ && !found) {
      std::stringstream ss;
      ss << "missing param " << impl_->name() << ". defaulting to ";
      config_parser::displayParam(ss, value);
      ss << std::endl;

      logger_->log_missing(ss.str());
    }
  }

 private:
  std::unique_ptr<Impl> impl_;
  Logger::Ptr logger_;
};

template <typename T>
struct is_parser<Parser<T>> : std::true_type {};

}  // namespace config_parser
