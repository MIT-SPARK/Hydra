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
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "hydra/config/config_visitor.h"
#include "hydra/config/ostream_formatter.h"

namespace config_parser {

struct Logger {
  using Ptr = std::shared_ptr<Logger>;

  virtual ~Logger() = default;

  virtual void log_missing(const std::string& message) const = 0;

  virtual void log_invalid(const std::string& message) const {
    std::cerr << "[Parsing Error]: " << message << std::endl;
  }
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
    const bool found = impl_->parse(value, logger_.get());
    if (logger_ && !found) {
      std::stringstream ss;
      ss << "missing param " << impl_->name() << ". defaulting to ";
      config_parser::displayParam(ss, value);

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
