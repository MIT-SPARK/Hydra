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
#include "hydra/common/node_matchers.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {
namespace {

static const auto bbox_registration =
    config::RegistrationWithConfig<NodeMatcher,
                                   CentroidBBoxMatcher,
                                   CentroidBBoxMatcher::Config>("CentroidBBoxMatcher");

static const auto iou_registration =
    config::RegistrationWithConfig<NodeMatcher, IoUNodeMatcher, IoUNodeMatcher::Config>(
        "CentroidBBoxMatcher");

}  // namespace

using namespace spark_dsg;

bool CentroidBBoxMatcher::match(const Attrs& new_attrs, const Attrs& prev_attrs) const {
  const auto new_derived = dynamic_cast<const SemanticNodeAttributes*>(&new_attrs);
  const auto prev_derived = dynamic_cast<const SemanticNodeAttributes*>(&prev_attrs);
  if (!new_derived || !prev_derived) {
    LOG(WARNING) << "Unable to cast both attributes to SemanticNodeAttributes";
    return false;
  }

  return prev_derived->bounding_box.contains(new_derived->position) ||
         new_derived->bounding_box.contains(prev_derived->position);
}

void declare_config(CentroidBBoxMatcher::Config&) {
  using namespace config;
  name("CentroidBBoxMatcher::Config");
}

IoUNodeMatcher::IoUNodeMatcher(const Config& config)
    : config(config::checkValid(config)) {}

bool IoUNodeMatcher::match(const Attrs& new_attrs, const Attrs& prev_attrs) const {
  const auto new_derived = dynamic_cast<const SemanticNodeAttributes*>(&new_attrs);
  const auto prev_derived = dynamic_cast<const SemanticNodeAttributes*>(&prev_attrs);
  if (!new_derived || !prev_derived) {
    LOG(WARNING) << "Unable to cast both attributes to SemanticNodeAttributes";
    return false;
  }

  // TODO(nathan) label vs. feature, logging
  const auto iou = new_derived->bounding_box.computeIoU(prev_derived->bounding_box);
  const auto threshold = new_derived->semantic_label == prev_derived->semantic_label
                             ? config.min_same_iou
                             : config.min_cross_iou;
  return iou >= threshold;
}

void declare_config(IoUNodeMatcher::Config& config) {
  using namespace config;
  name("IoUNodeMatcher::Config");
  field(config.min_cross_iou, "min_cross_iou");
  field(config.min_same_iou, "min_same_iou");
  check(config.min_cross_iou, GT, 0.0, "min_cross_iou");
  check(config.min_same_iou, GT, 0.0, "min_same_iou");
}

}  // namespace hydra
