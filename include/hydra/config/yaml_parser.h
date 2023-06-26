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
#include <yaml-cpp/yaml.h>

#include <vector>

#include "hydra/config/config_parser.h"

namespace config_parser {

class YamlParserImpl {
 public:
  YamlParserImpl() = default;

  YamlParserImpl(const YamlParserImpl& other) = default;

  explicit YamlParserImpl(const std::string& file);

  explicit YamlParserImpl(const YAML::Node& node);

  YamlParserImpl& operator=(const YamlParserImpl& other) = default;

  ~YamlParserImpl() = default;

  YamlParserImpl child(const std::string& new_name) const;

  std::vector<std::string> children() const;

  inline std::string name() const { return name_; }

  template <typename T>
  bool parse(T& value, const Logger* logger) const {
    if (!node_) {
      return false;
    }

    try {
      return parseImpl(value);
    } catch (const std::exception& e) {
      if (logger) {
        std::stringstream ss;
        ss << "failed to parse " << name_ << ": " << e.what();
        logger->log_invalid(ss.str());
      }
      return false;
    }
  }

 private:
  YamlParserImpl(const YAML::Node& node, const std::string& name);

  template <typename T,
            typename std::enable_if<std::negation<is_config_enum<T>>::value,
                                    bool>::type = true>
  bool parseImpl(T& value) const {
    value = node_.as<T>();
    return true;
  }

  template <typename T,
            typename std::enable_if<is_config_enum<T>::value, bool>::type = true>
  bool parseImpl(T& value) const {
    const auto placeholder = node_.as<std::string>();
    readConfigEnumFromString(placeholder, value);
    return true;
  }

  template <typename T>
  bool parseImpl(std::vector<T>& value) const {
    if (!node_.IsSequence()) {
      return false;
    }

    value = node_.as<std::vector<T>>();
    return true;
  }

  template <typename T>
  bool parseImpl(std::set<T>& value) const {
    if (!node_.IsSequence()) {
      return false;
    }

    std::vector<T> placeholder = node_.as<std::vector<T>>();
    value.clear();
    value.insert(placeholder.begin(), placeholder.end());
    return true;
  }

  template <typename K, typename V>
  bool parseImpl(std::map<K, V>& value) const {
    if (!node_.IsMap()) {
      return false;
    }

    value = node_.as<std::map<K, V>>();
    return true;
  }

  bool parseImpl(uint8_t& value) const;

  YAML::Node node_;
  std::string name_;
};

using YamlParser = Parser<YamlParserImpl>;

}  // namespace config_parser
