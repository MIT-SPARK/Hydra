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
#include "hydra/gnn/tensor.h"

#include <numeric>

namespace hydra::gnn {

size_t getTypeSize(Tensor::Type type) {
  switch (type) {
    case Tensor::Type::FLOAT32:
      return sizeof(float);
    case Tensor::Type::FLOAT64:
      return sizeof(double);
    case Tensor::Type::INT8:
      return sizeof(int8_t);
    case Tensor::Type::INT16:
      return sizeof(int16_t);
    case Tensor::Type::INT32:
      return sizeof(int32_t);
    case Tensor::Type::INT64:
      return sizeof(int64_t);
    case Tensor::Type::UINT8:
      return sizeof(uint8_t);
    case Tensor::Type::UINT16:
      return sizeof(uint16_t);
    case Tensor::Type::UINT32:
      return sizeof(uint32_t);
    case Tensor::Type::UINT64:
      return sizeof(uint64_t);
    default:
      // TODO(nathan) throw exception?
      return 0;
  }
}

Tensor::Tensor() : Tensor(Tensor::Type::FLOAT32) {}

Tensor::Tensor(Tensor::Type type) : Tensor({}, type) {}

Tensor::Tensor(int64_t rows, int64_t cols)
    : Tensor(rows, cols, Tensor::Type::FLOAT32) {}

Tensor::Tensor(int64_t rows, int64_t cols, Tensor::Type type)
    : Tensor({rows, cols}, type) {}

Tensor::Tensor(const std::vector<int64_t>& dims, Type type) : type_(type), dims_(dims) {
  if (dims.size() == 0) {
    size_ = 0;
  } else {
    size_ = std::accumulate(dims.begin(), dims.end(), 1, std::multiplies<size_t>());
  }

  memory_.reset(new std::vector<char>(size_ * getTypeSize(type)));
}

Tensor::operator bool() const { return size_ != 0; }

Tensor::Type Tensor::type() const { return type_; }

size_t Tensor::size() const { return size_; }

size_t Tensor::num_bytes() const { return memory_->size(); }

const std::vector<int64_t>& Tensor::dims() const { return dims_; }

size_t Tensor::num_dims() const { return dims_.size(); }

int64_t Tensor::rows() const { return dims_.empty() ? 0 : dims_[0]; }

int64_t Tensor::cols() const {
  if (dims_.empty()) {
    return 0;
  }

  if (dims_.size() < 2) {
    return 1;
  }

  return dims_[1];
}

std::string getTensorTypeStr(Tensor::Type type) {
  switch (type) {
    case Tensor::Type::FLOAT32:
      return "float32";
    case Tensor::Type::FLOAT64:
      return "float64";
    case Tensor::Type::UINT8:
      return "uint8";
    case Tensor::Type::UINT16:
      return "uint16";
    case Tensor::Type::UINT32:
      return "uint32";
    case Tensor::Type::UINT64:
      return "uint64";
    case Tensor::Type::INT8:
      return "int8";
    case Tensor::Type::INT16:
      return "int16";
    case Tensor::Type::INT32:
      return "int32";
    case Tensor::Type::INT64:
      return "int64";
    default:
      return "unknown";
  }
}

std::ostream& operator<<(std::ostream& out, const Tensor& tensor) {
  out << getTensorTypeStr(tensor.type()) << "[";

  const auto dims = tensor.dims();

  auto iter = dims.begin();
  while (iter != dims.end()) {
    out << *iter;
    ++iter;

    if (iter != dims.end()) {
      out << " x ";
    }
  }

  out << "]";
  return out;
}

}  // namespace::hydra
