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
#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>

namespace hydra {

enum class SensorMapMode {
  //! Config is not namespaced by sensor
  Single,
  //! Config is namespaced by sensor. If sensor name is not matched, throw error
  Strict,
  //! Config is namespaced by sensor. If sensor name is not matched, use default
  Permissive,
};

// NOTE(nathan) this is intentionally separate from SensorMap to make template argument
// deduction work (an argument of `SensorMap<T>::Config&` will not participate in
// template argument deduction)
template <typename T>
struct SensorMapConfig {
  SensorMapMode mode = SensorMapMode::Single;
  config::OrderedMap<std::string, typename T::Config> sensors;
};

template <typename T>
class SensorMap {
 public:
  using Config = SensorMapConfig<T>;

  explicit SensorMap(const Config& config) : config(config) {}
  T* get(const std::string& name);
  const T* get(const std::string& name) const;

  const Config config;

 private:
  mutable std::map<std::string, std::unique_ptr<T>> elements_;
};

template <typename T>
struct SensorMapConversion {
  static T toIntermediate(const config::OrderedMap<std::string, T>& value,
                          std::string& error) {
    if (value.size() > 1u) {
      error = "Map does not have a single element";
      return {};
    }

    return value.empty() ? T{} : value.front().second;
  }

  static void fromIntermediate(const T& intermediate,
                               config::OrderedMap<std::string, T>& value,
                               std::string& error) {
    if (value.size() > 1u) {
      error = "Map does not have a single element";
      return;
    }

    if (value.empty()) {
      value.emplace_back(std::make_pair("", intermediate));
    } else {
      value[0].second = intermediate;
    }
  }
};

template <typename T>
T* SensorMap<T>::get(const std::string& name) {
  auto iter = elements_.find(name);
  if (iter == elements_.end()) {
    std::optional<typename T::Config> sensor_config;
    for (const auto& [_name, candidate] : config.sensors) {
      if (_name.empty() || _name == name) {
        sensor_config = candidate;
        break;
      }
    }

    if (!sensor_config) {
      switch (config.mode) {
        case SensorMapMode::Single:
          throw std::runtime_error("single mode must always have config!");
        case SensorMapMode::Permissive:
          sensor_config = typename T::Config();
          break;
        case SensorMapMode::Strict:
        default:
          return nullptr;
      }
    }

    iter = elements_.emplace(name, std::make_unique<T>(*sensor_config)).first;
  }

  return iter->second.get();
}

template <typename T>
const T* SensorMap<T>::get(const std::string& name) const {
  return const_cast<SensorMap<T>*>(this)->get(name);
}

template <typename T>
void declare_config(SensorMapConfig<T>& config) {
  using namespace config;
  name<SensorMapConfig<T>>();
  enum_field(config.mode,
             "sensor_map_mode",
             {{SensorMapMode::Single, "Single"},
              {SensorMapMode::Strict, "Strict"},
              {SensorMapMode::Permissive, "Permissive"}});
  if (config.mode == SensorMapMode::Single) {
    field<SensorMapConversion<typename T::Config>>(config.sensors, "sensors");
  } else {
    field(config.sensors, "sensors");
  }
}

}  // namespace hydra
