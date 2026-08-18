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
#include "hydra_visualizer/filters/node_filters.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <spark_dsg/node_attributes.h>

namespace hydra {
namespace {

static const auto real_place_reg =
    config::RegistrationWithConfig<NodeFilter,
                                   RealPlaceFilter,
                                   RealPlaceFilter::Config>("RealPlaceFilter");

}

void declare_config(RealPlaceFilter::Config& config) {
  using namespace config;
  name("RealPlaceFilter::Config");
  field(config.real_is_valid, "real_is_valid");
  field(config.place_attributes_required, "place_attributes_required");
}

RealPlaceFilter::RealPlaceFilter(const Config& config) : config(config) {}

bool RealPlaceFilter::valid(const spark_dsg::SceneGraph&,
                            const spark_dsg::SceneGraphNode& node) const {
  const auto attrs = node.tryAttributes<spark_dsg::PlaceNodeAttributes>();
  if (!attrs) {  // still draw non-place nodes if flags are set correctly
    return !config.place_attributes_required && !config.real_is_valid;
  }

  return config.real_is_valid ? attrs->real_place : !attrs->real_place;
}

}  // namespace hydra
