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
#include "hydra/frontend/keyframe_gate.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>

#include <cmath>

namespace hydra {

void declare_config(KeyframeGate::Config& config) {
  using namespace config;
  name("KeyframeGate::Config");
  field(config.min_translation_m, "min_translation_m", "m");
  field(config.min_rotation_deg, "min_rotation_deg", "deg");
  check(config.min_translation_m, GE, 0.0, "min_translation_m");
  check(config.min_rotation_deg, GE, 0.0, "min_rotation_deg");
}

bool KeyframeGate::shouldTrigger(const Eigen::Vector3d& position,
                                 const Eigen::Quaterniond& orientation) {
  if (initialized_) {
    const double translation_diff = (position - last_position_).norm();
    const double angular_diff =
        last_orientation_.angularDistance(orientation) * 180.0 / M_PI;
    if (translation_diff < config_.min_translation_m &&
        angular_diff < config_.min_rotation_deg) {
      return false;
    }
  }

  last_position_ = position;
  last_orientation_ = orientation;
  initialized_ = true;
  return true;
}

}  // namespace hydra
