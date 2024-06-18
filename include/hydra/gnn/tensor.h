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
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

namespace hydra::gnn {

// forward declare for output operator
class Tensor;

std::ostream& operator<<(std::ostream& out, const Tensor& tensor);

class Tensor {
 public:
  template <typename T>
  using MapTemplate =
      Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

  // note that this is a subset of what onnx actually supports, but the others (e.g.,
  // float16 or complex64) aren't as trivially representable with eigen
  enum class Type {
    FLOAT32,
    FLOAT64,
    INT8,
    INT16,
    INT32,
    INT64,
    UINT8,
    UINT16,
    UINT32,
    UINT64,
  };

  template <typename T>
  static inline bool matchesType([[maybe_unused]] Type type,
                                 [[maybe_unused]] T* dummy = nullptr) {
    return false;
  }

  Tensor();

  explicit Tensor(Type type);

  Tensor(int64_t rows, int64_t cols);

  Tensor(int64_t rows, int64_t cols, Type type);

  Tensor(const std::vector<int64_t>& dims, Type type);

  ~Tensor() = default;

  operator bool() const;

  Type type() const;

  size_t size() const;

  size_t num_bytes() const;

  const std::vector<int64_t>& dims() const;

  size_t num_dims() const;

  int64_t rows() const;

  int64_t cols() const;

  template <typename T>
  const T* data(bool validate = true) const {
    if (validate && !matchesType<T>(type_)) {
      throw std::invalid_argument("invalid data conversion for tensor");
    }

    return reinterpret_cast<const T*>(memory_->data());
  }

  template <typename T>
  T* data(bool validate = true) {
    return const_cast<T*>(const_cast<const Tensor*>(this)->data<T>(validate));
  }

  template <typename T>
  MapTemplate<T> map(bool validate = true) {
    if (validate && dims_.size() > 2) {
      throw std::invalid_argument("cannot map data to 2D matrix");
    }

    return MapTemplate<T>(data<T>(validate), rows(), cols());
  }

 private:
  Type type_;
  std::shared_ptr<std::vector<char>> memory_;
  std::vector<int64_t> dims_;
  size_t size_;
};

template <>
inline bool Tensor::matchesType(Tensor::Type type, float*) {
  return type == Tensor::Type::FLOAT32;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, double*) {
  return type == Tensor::Type::FLOAT64;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, uint8_t*) {
  return type == Tensor::Type::UINT8;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, uint16_t*) {
  return type == Tensor::Type::UINT16;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, uint32_t*) {
  return type == Tensor::Type::UINT32;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, uint64_t*) {
  return type == Tensor::Type::UINT64;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, int8_t*) {
  return type == Tensor::Type::INT8;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, int16_t*) {
  return type == Tensor::Type::INT16;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, int32_t*) {
  return type == Tensor::Type::INT32;
}

template <>
inline bool Tensor::matchesType(Tensor::Type type, int64_t*) {
  return type == Tensor::Type::INT64;
}

}  // namespace::hydra
