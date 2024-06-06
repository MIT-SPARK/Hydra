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
#include <config_utilities/virtual_config.h>

#include <list>
#include <memory>
#include <string>

namespace hydra {

template <typename... Args>
struct OutputSink {
  using Sink = OutputSink<Args...>;
  using Ptr = std::shared_ptr<Sink>;
  using Factory = config::VirtualConfig<Sink>;
  using List = std::list<Ptr>;

  virtual ~OutputSink() = default;
  virtual void call(Args... args) const = 0;
  virtual std::string printInfo() const { return ""; }

  static Ptr fromCallback(const std::function<void(Args...)>& callback);

  template <typename T>
  static Ptr fromMethod(void (T::*callback)(Args...) const, const T* instance);

  template <typename T>
  static Ptr fromMethod(void (T::*callback)(Args...), T* instance);

  static List instantiate(const std::vector<Factory>& configs) {
    List sinks;
    for (const auto& config : configs) {
      if (config) {
        sinks.push_back(config.create());
      }
    }
    return sinks;
  }

  static void callAll(const List& sinks, Args... args) {
    for (const auto& sink : sinks) {
      if (sink) {
        sink->call(args...);
      }
    }
  }
};

template <typename... Args>
struct FunctionSink : OutputSink<Args...> {
  FunctionSink(const std::function<void(Args...)>& f) : func(f) {}
  virtual ~FunctionSink() = default;

  void call(Args... args) const override { func(args...); }

  std::function<void(Args...)> func;
};

template <typename... Args>
typename OutputSink<Args...>::Ptr OutputSink<Args...>::fromCallback(
    const std::function<void(Args...)>& callback) {
  return std::make_shared<FunctionSink<Args...>>(callback);
}

template <typename T, typename... Args>
struct MethodSink : OutputSink<Args...> {
  MethodSink(void (T::*callback)(Args...) const, const T* instance)
      : callback(callback), instance(instance) {}

  void call(Args... args) const override { (instance->*callback)(args...); }

  void (T::*callback)(Args...) const;
  const T* instance;
};

template <typename... Args>
template <typename T>
typename OutputSink<Args...>::Ptr OutputSink<Args...>::fromMethod(
    void (T::*callback)(Args...) const, const T* instance) {
  return std::make_shared<MethodSink<T, Args...>>(callback, instance);
}

template <typename T, typename... Args>
struct NonConstMethodSink : OutputSink<Args...> {
  NonConstMethodSink(void (T::*callback)(Args...), T* instance)
      : callback(callback), instance(instance) {}

  void call(Args... args) const override { (instance->*callback)(args...); }

  void (T::*callback)(Args...);
  T* instance;
};

template <typename... Args>
template <typename T>
typename OutputSink<Args...>::Ptr OutputSink<Args...>::fromMethod(
    void (T::*callback)(Args...), T* instance) {
  return std::make_shared<NonConstMethodSink<T, Args...>>(callback, instance);
}

}  // namespace hydra
