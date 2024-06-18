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
#include "hydra/gnn/ort_utilities.h"

#include <functional>
#include <sstream>

namespace hydra::gnn {

bool FieldInfo::tensorMatchesType(const Tensor& tensor) const {
  switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
      return tensor.type() == Tensor::Type::FLOAT32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
      return tensor.type() == Tensor::Type::UINT8;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
      return tensor.type() == Tensor::Type::INT8;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
      return tensor.type() == Tensor::Type::UINT16;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
      return tensor.type() == Tensor::Type::INT16;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
      return tensor.type() == Tensor::Type::INT32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
      return tensor.type() == Tensor::Type::INT64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
      return tensor.type() == Tensor::Type::FLOAT64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
      return tensor.type() == Tensor::Type::UINT32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
      return tensor.type() == Tensor::Type::UINT64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX64:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX128:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
    default:
      return false;
  }
}

void FieldInfo::validateTensor(const Tensor& tensor) const {
  if (dims.size() != tensor.dims().size()) {
    std::stringstream ss;
    ss << "tensor dimension size (" << tensor.dims().size()
       << ") does not match field dimension size (" << dims.size() << ") for field "
       << name;
    throw std::invalid_argument(ss.str());
  }

  for (size_t i = 0; i < dims.size(); ++i) {
    if (dims[i] == -1) {
      continue;
    }

    if (dims[i] != tensor.dims()[i]) {
      std::stringstream ss;
      ss << "provided tensor dimension " << i << " (size " << tensor.dims()[i]
         << ") does not match field (size " << dims[i] << ") for field " << name;
      throw std::invalid_argument(ss.str());
    }
  }

  if (!tensorMatchesType(tensor)) {
    std::stringstream ss;
    ss << "provided tensor " << tensor << "does not have type " << type_name;
  }
}

template <typename T>
Ort::Value makeOrtTensor(const OrtMemoryInfo* mem_info, const Tensor& input) {
  return Ort::Value::CreateTensor<T>(mem_info,
                                     const_cast<T*>(input.data<T>()),
                                     input.size(),
                                     input.dims().data(),
                                     input.dims().size());
}

Ort::Value FieldInfo::makeOrtValue(const OrtMemoryInfo* mem_info,
                                   const Tensor& input,
                                   bool validate) const {
  if (!mem_info) {
    throw std::domain_error("invalid memory info");
  }

  if (validate) {
    validateTensor(input);
  }

  switch (input.type()) {
    case Tensor::Type::FLOAT32:
      return makeOrtTensor<float>(mem_info, input);
    case Tensor::Type::FLOAT64:
      return makeOrtTensor<double>(mem_info, input);
    case Tensor::Type::UINT8:
      return makeOrtTensor<uint8_t>(mem_info, input);
    case Tensor::Type::UINT16:
      return makeOrtTensor<uint16_t>(mem_info, input);
    case Tensor::Type::UINT32:
      return makeOrtTensor<uint32_t>(mem_info, input);
    case Tensor::Type::UINT64:
      return makeOrtTensor<uint64_t>(mem_info, input);
    case Tensor::Type::INT8:
      return makeOrtTensor<int8_t>(mem_info, input);
    case Tensor::Type::INT16:
      return makeOrtTensor<int16_t>(mem_info, input);
    case Tensor::Type::INT32:
      return makeOrtTensor<int32_t>(mem_info, input);
    case Tensor::Type::INT64:
      return makeOrtTensor<int64_t>(mem_info, input);
    default:
      throw std::invalid_argument("tensor type is unhandled!");
  }
}

std::optional<Tensor::Type> OrtToTensorType(ONNXTensorElementDataType type) {
  switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
      return Tensor::Type::FLOAT32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
      return Tensor::Type::UINT8;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
      return Tensor::Type::INT8;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
      return Tensor::Type::UINT16;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
      return Tensor::Type::INT16;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
      return Tensor::Type::INT32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
      return Tensor::Type::INT64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
      return Tensor::Type::FLOAT64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
      return Tensor::Type::UINT32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
      return Tensor::Type::UINT64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX64:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX128:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
    default:
      return std::nullopt;
  }
}

Tensor FieldInfo::getTensor() const {
  for (const auto dim_size : dims) {
    if (dim_size == -1) {
      std::stringstream ss;
      ss << "cannot make a tensor with dynamic dimensions"
         << " for field " << name;
      throw std::invalid_argument(ss.str());
    }
  }

  auto tensor_type = OrtToTensorType(type);
  if (!tensor_type) {
    std::stringstream ss;
    ss << "cannot make tensor of corresponding type to " << type_name << " for field "
       << name;
    throw std::invalid_argument(ss.str());
  }

  return Tensor(dims, *tensor_type);
}

Tensor FieldInfo::getDynamicTensor(const std::vector<size_t>& dims_to_read,
                                   const std::vector<int64_t>& output_dims) const {
  size_t dynamic_index = 0;
  std::vector<int64_t> new_dims;
  for (size_t i = 0; i < dims.size(); ++i) {
    if (dims[i] != -1) {
      new_dims.push_back(dims[i]);
      continue;
    }

    if (dynamic_index >= dims_to_read.size()) {
      std::stringstream ss;
      ss << "dynamic size required for axis " << i << " for field " << name;
      throw std::invalid_argument(ss.str());
    }

    const auto output_index = dims_to_read[dynamic_index];
    ++dynamic_index;
    if (output_index >= output_dims.size()) {
      std::stringstream ss;
      ss << "output dimensions missing index " << output_index << " for field " << name;
      throw std::invalid_argument(ss.str());
    }

    const auto new_dim = output_dims[output_index];
    if (new_dim < 0) {
      std::stringstream ss;
      ss << "invalid output dimension at index " << output_index << ": " << new_dim;
      throw std::invalid_argument(ss.str());
    }

    new_dims.push_back(new_dim);
  }

  auto tensor_type = OrtToTensorType(type);
  if (!tensor_type) {
    std::stringstream ss;
    ss << "cannot make tensor of corresponding type to " << type_name << " for field "
       << name;
    throw std::invalid_argument(ss.str());
  }

  return Tensor(new_dims, *tensor_type);
}

using SizeGetter = std::function<size_t(const Ort::Session*)>;
using NameGetter =
    std::function<Ort::AllocatedStringPtr(const Ort::Session*, size_t, OrtAllocator*)>;
using InfoGetter = std::function<Ort::TypeInfo(const Ort::Session*, size_t)>;

std::vector<FieldInfo> readFieldInfo(const Ort::Session* session,
                                     OrtAllocator* allocator,
                                     const SizeGetter& size_getter,
                                     const NameGetter& name_getter,
                                     const InfoGetter& info_getter) {
  if (!session || !allocator) {
    return {};
  }

  std::vector<FieldInfo> fields;
  const size_t num_fields = size_getter(session);
  for (size_t i = 0; i < num_fields; ++i) {
    auto str_ptr = name_getter(session, i, allocator);
    if (!str_ptr) {
      throw std::domain_error("invalid name for field found!");
    }

    const auto type_info = info_getter(session, i);
    const auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    const auto type = tensor_info.GetElementType();

    fields.push_back({std::string(str_ptr.get()),
                      tensor_info.GetShape(),
                      type,
                      getElementTypeString(type)});
  }

  return fields;
}

std::vector<FieldInfo> getSessionInputs(const Ort::Session* session,
                                        OrtAllocator* allocator) {
  return readFieldInfo(session,
                       allocator,
                       &Ort::Session::GetInputCount,
                       &Ort::Session::GetInputNameAllocated,
                       &Ort::Session::GetInputTypeInfo);
}

std::vector<FieldInfo> getSessionOutputs(const Ort::Session* session,
                                         OrtAllocator* allocator) {
  return readFieldInfo(session,
                       allocator,
                       &Ort::Session::GetOutputCount,
                       &Ort::Session::GetOutputNameAllocated,
                       &Ort::Session::GetOutputTypeInfo);
}

std::string getElementTypeString(ONNXTensorElementDataType type) {
  switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED:
      return "undefined";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
      return "float";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
      return "uint8";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
      return "int8";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
      return "uint16";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
      return "int16";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
      return "int32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
      return "int64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
      return "string";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
      return "bool";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
      return "float16";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
      return "double";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
      return "uint32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
      return "uint64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX64:
      return "complex64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX128:
      return "complex128";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
      return "bfloat16";
    default:
      return "invalid";
  }
}

std::ostream& operator<<(std::ostream& out, const FieldInfo& info) {
  out << info.name << " -> " << info.type_name << "[";

  auto iter = info.dims.begin();
  while (iter != info.dims.end()) {
    if (*iter == -1) {
      out << "N";
    } else {
      out << *iter;
    }

    ++iter;

    if (iter != info.dims.end()) {
      out << " x ";
    }
  }
  out << "]";
  return out;
}

}  // namespace::hydra
