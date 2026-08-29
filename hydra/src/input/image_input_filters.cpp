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

#include "hydra/input/image_input_filters.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<InputFilter,
                                   InvalidDepthFilter,
                                   InvalidDepthFilter::Config>("InvalidDepthFilter");

}

void declare_config(InvalidDepthFilter::Config& config) {
  using namespace config;
  name("InvalidDepthFilter::Config");
  field(config.max_invalid_ratio, "max_invalid_ratio");
  checkInRange(config.max_invalid_ratio, 0.0, 1.0, "max_invalid_ratio", false);
}

InvalidDepthFilter::InvalidDepthFilter(const Config& config)
    : config(config::checkValid(config)) {}

bool InvalidDepthFilter::valid(const SensorInputPacket& current,
                               const SensorInputPacket* const) const {
  const auto derived = dynamic_cast<const ImageInputPacket* const>(&current);
  if (!derived) {
    return true;
  }

  const auto& depth = derived->depth;
  if (depth.empty() && depth.type() != CV_32FC1) {
    return true;
  }

  const size_t total = depth.rows * depth.cols;
  size_t num_invalid = 0;
  for (int r = 0; r < depth.rows; r++) {
    for (int c = 0; c < depth.cols; c++) {
      const auto value = depth.at<float>(r, c);
      num_invalid += std::isfinite(value) ? 0 : 1;
    }
  }

  const auto ratio = static_cast<double>(num_invalid) / total;
  return ratio < config.max_invalid_ratio;
}

}  // namespace hydra
