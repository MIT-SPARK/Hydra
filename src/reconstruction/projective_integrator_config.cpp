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
#include "hydra/reconstruction/projective_integrator_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>

namespace hydra {

void declare_config(ProjectiveIntegratorConfig& config) {
  using namespace config;
  name("ProjectiveIntegrator");
  field(config.verbosity, "verbosity");
  // TODO(nathan) push to sensor
  field(config.use_weight_dropoff, "use_weight_dropoff");
  field(config.weight_dropoff_epsilon,
        "weight_dropoff_epsilon",
        config.weight_dropoff_epsilon >= 0 ? "m" : "vs");
  field(config.use_constant_weight, "use_constant_weight");
  field(config.max_weight, "max_weight");
  field<ThreadNumConversion>(config.num_threads, "num_threads");
  field(config.interp_method, "interpolation_method");
  config.semantic_integrator.setOptional();
  field(config.semantic_integrator, "semantic_integrator");

  check(config.num_threads, GT, 0, "num_threads");
  check(config.max_weight, GT, 0, "max_weight");
  if (config.use_weight_dropoff) {
    check(config.weight_dropoff_epsilon, NE, 0.f, "weight_dropoff_epsilon");
  }
}

}  // namespace hydra
