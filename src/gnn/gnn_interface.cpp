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
#include "hydra/gnn/gnn_interface.h"

#include <glog/logging.h>

#include "hydra/gnn/ort_utilities.h"

namespace hydra::gnn {

struct GnnInterfaceImpl {
  explicit GnnInterfaceImpl(const std::string& model_path)
      : GnnInterfaceImpl(model_path, {}) {}

  GnnInterfaceImpl(const std::string& model_path, const DynamicIndexMap& output_map)
      : model_path(model_path),
        output_map(output_map),
        mem_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                            OrtMemType::OrtMemTypeDefault)) {
    env.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "hydra_gnn_interface"));
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1).SetInterOpNumThreads(1);
    session.reset(new Ort::Session(*env, model_path.c_str(), options));
    allocator.reset(new Ort::AllocatorWithDefaultOptions());
    mem_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                          OrtMemType::OrtMemTypeDefault);

    inputs = getSessionInputs(session.get(), *allocator);
    for (const auto& input : inputs) {
      input_names.push_back(input.name.c_str());
    }

    outputs = getSessionOutputs(session.get(), *allocator);
    for (const auto& output : outputs) {
      output_names.push_back(output.name.c_str());
    }
  }

  TensorMap operator()(const TensorMap& tensors,
                       const std::vector<int64_t>& output_dimensions) const {
    std::vector<Ort::Value> input_values;
    for (const auto& input : inputs) {
      const auto iter = tensors.find(input.name);
      if (iter == tensors.end()) {
        std::stringstream ss;
        ss << "missing input " << input.name << " from provided input tensors!";
        throw std::invalid_argument(ss.str());
      }

      input_values.push_back(input.makeOrtValue(mem_info, iter->second));
    }

    TensorMap tensor_outputs;
    std::vector<Ort::Value> output_values;
    for (const auto& output : outputs) {
      const auto iter = output_map.find(output.name);

      Tensor tensor;
      if (iter == output_map.end()) {
        tensor = output.getTensor();
      } else {
        tensor = output.getDynamicTensor(iter->second, output_dimensions);
      }

      tensor_outputs[output.name] = tensor;
      output_values.push_back(output.makeOrtValue(mem_info, tensor));
    }

    session->Run(Ort::RunOptions(nullptr),
                 input_names.data(),
                 input_values.data(),
                 input_names.size(),
                 output_names.data(),
                 output_values.data(),
                 output_names.size());

    return tensor_outputs;
  }

  std::string model_path;

  std::vector<FieldInfo> inputs;
  std::vector<const char*> input_names;
  std::vector<FieldInfo> outputs;
  std::vector<const char*> output_names;

  DynamicIndexMap output_map;

  std::unique_ptr<Ort::Env> env;
  std::unique_ptr<Ort::AllocatorWithDefaultOptions> allocator;
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem_info;
};

std::ostream& operator<<(std::ostream& out, const GnnInterfaceImpl& impl) {
  out << "model_path: " << impl.model_path << std::endl;

  out << "inputs: " << std::endl;
  for (const auto& input : impl.inputs) {
    out << "  - " << input << std::endl;
  }

  out << "outputs: " << std::endl;
  for (const auto& output : impl.outputs) {
    out << "  - " << output << std::endl;
  }

  out << "dynamic output axes: " << std::endl;
  for (const auto& name_index_pair : impl.output_map) {
    out << " - " << name_index_pair.first << ": [";
    auto iter = name_index_pair.second.begin();
    while (iter != name_index_pair.second.end()) {
      out << *iter;
      ++iter;
      if (iter != name_index_pair.second.end()) {
        out << ", ";
      }
    }
    out << "]" << std::endl;
  }

  return out;
}

GnnInterface::GnnInterface(const std::string& model_path)
    : GnnInterface(model_path, {}) {}

GnnInterface::GnnInterface(const std::string& model_path,
                           const DynamicIndexMap& output_map)
    : model_path_(model_path) {
  impl_.reset(new GnnInterfaceImpl(model_path, output_map));
}

GnnInterface::~GnnInterface() {}

Tensor GnnInterface::operator()(const Tensor& x, const Tensor& edge_index) const {
  auto map = (*impl_)({{"x", x}, {"edge_index", edge_index}}, {});
  return map.at("output");
}

TensorMap GnnInterface::operator()(const TensorMap& input) const {
  return (*impl_)(input, {});
}

TensorMap GnnInterface::operator()(const TensorMap& input,
                                   const std::vector<int64_t>& output_sizes) const {
  return (*impl_)(input, output_sizes);
}

std::ostream& operator<<(std::ostream& out, const GnnInterface& gnn) {
  out << "providers: ";
  const auto providers = Ort::GetAvailableProviders();

  out << "[";
  auto iter = providers.begin();
  while (iter != providers.end()) {
    out << *iter;

    ++iter;

    if (iter != providers.end()) {
      out << ", ";
    }
  }
  out << "]" << std::endl;

  if (gnn.impl_) {
    out << *gnn.impl_;
  } else {
    out << "model_path: n/a";
  }

  return out;
}

}  // namespace::hydra
