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
#include <map>
#include <memory>
#include <string>

#include "hydra/gnn/tensor.h"

namespace hydra::gnn {

using TensorMap = std::map<std::string, Tensor>;
using DynamicIndexMap = std::map<std::string, std::vector<size_t>>;

struct GnnInterfaceImpl;

class GnnInterface {
 public:
  // TODO(nathan) add actual config
  explicit GnnInterface(const std::string& model_path);

  GnnInterface(const std::string& model_path, const DynamicIndexMap& output_map);

  ~GnnInterface();

  /**
   * \brief Run inference with node features and edge indices
   *
   * Note that this assumes the model was exported with the same order of inputs:
   * x, then edge_index!
   *
   * \param[in] x Node features
   * \param[in] edge_index Edge indices
   * \returns Result as tensor
   */
  Tensor operator()(const Tensor& x, const Tensor& edge_index) const;

  /**
   * \brief Run inference with abitrary input arguments
   *
   * \param[in] input Map between input name and Tensor value
   * \returns Result as tensor
   */
  TensorMap operator()(const TensorMap& input) const;

  /**
   * \brief Run inference with abitrary input arguments
   *
   * \param[in] input Map between input name and Tensor value
   * \param[in] output_sizes Output sizes for dynamic output axes
   * \returns Result as tensor
   */
  TensorMap operator()(const TensorMap& input,
                       const std::vector<int64_t>& output_sizes) const;

 protected:
  std::unique_ptr<GnnInterfaceImpl> impl_;

  std::string model_path_;

  /**
   * \brief Show information about the underlying model and deployment framework
   */
  friend std::ostream& operator<<(std::ostream& out, const GnnInterface& gnn);
};

}  // namespace::hydra
