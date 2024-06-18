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
#include <onnxruntime_cxx_api.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "hydra/gnn/tensor.h"

namespace hydra::gnn {

struct FieldInfo {
  std::string name;
  std::vector<int64_t> dims;
  ONNXTensorElementDataType type;
  std::string type_name;

  bool tensorMatchesType(const Tensor& tensor) const;

  void validateTensor(const Tensor& tensor) const;

  Ort::Value makeOrtValue(const OrtMemoryInfo* mem_info,
                          const Tensor& input,
                          bool validate = true) const;

  Tensor getTensor() const;

  Tensor getDynamicTensor(const std::vector<size_t>& dims_to_read,
                          const std::vector<int64_t>& output_dims) const;
};

std::vector<FieldInfo> getSessionInputs(const Ort::Session* session,
                                        OrtAllocator* allocator);

std::vector<FieldInfo> getSessionOutputs(const Ort::Session* session,
                                         OrtAllocator* allocator);

std::string getElementTypeString(ONNXTensorElementDataType type);

std::ostream& operator<<(std::ostream& out, const FieldInfo& info);

}  // namespace::hydra
