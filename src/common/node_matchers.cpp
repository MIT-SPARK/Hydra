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
        "IoUNodeMatcher");

static const auto distance_registration =
    config::RegistrationWithConfig<NodeMatcher,
                                   DistanceNodeMatcher,
                                   DistanceNodeMatcher::Config>("DistanceNodeMatcher");

static const auto place_registration =
    config::RegistrationWithConfig<NodeMatcher,
                                   PlaceNodeMatcher,
                                   PlaceNodeMatcher::Config>("PlaceNodeMatcher");

}  // namespace

using namespace spark_dsg;

bool CentroidBBoxMatcher::match(const Attrs& lhs_attrs, const Attrs& rhs_attrs) const {
  const auto lhs_derived = dynamic_cast<const SemanticNodeAttributes*>(&lhs_attrs);
  const auto rhs_derived = dynamic_cast<const SemanticNodeAttributes*>(&rhs_attrs);
  if (!lhs_derived || !rhs_derived) {
    LOG(WARNING) << "Unable to cast both attributes to SemanticNodeAttributes";
    return false;
  }

  return rhs_derived->bounding_box.contains(lhs_derived->position) ||
         lhs_derived->bounding_box.contains(rhs_derived->position);
}

void declare_config(CentroidBBoxMatcher::Config&) {
  using namespace config;
  name("CentroidBBoxMatcher::Config");
}

IoUNodeMatcher::IoUNodeMatcher(const Config& config)
    : config(config::checkValid(config)) {}

bool IoUNodeMatcher::match(const Attrs& lhs_attrs, const Attrs& rhs_attrs) const {
  const auto lhs_derived = dynamic_cast<const SemanticNodeAttributes*>(&lhs_attrs);
  const auto rhs_derived = dynamic_cast<const SemanticNodeAttributes*>(&rhs_attrs);
  if (!lhs_derived || !rhs_derived) {
    LOG(WARNING) << "Unable to cast both attributes to SemanticNodeAttributes";
    return false;
  }

  // TODO(nathan) label vs. feature, logging
  const auto iou = lhs_derived->bounding_box.computeIoU(rhs_derived->bounding_box);
  const auto threshold = lhs_derived->semantic_label == rhs_derived->semantic_label
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

DistanceNodeMatcher::DistanceNodeMatcher(const Config& config)
    : config(config::checkValid(config)) {}

bool DistanceNodeMatcher::match(const Attrs& lhs_attrs, const Attrs& rhs_attrs) const {
  const auto distance = (lhs_attrs.position - rhs_attrs.position).norm();
  return distance <= config.pos_threshold_m;
}

void declare_config(DistanceNodeMatcher::Config& config) {
  using namespace config;
  name("DistanceNodeMatcher::Config");
  field(config.pos_threshold_m, "pos_threshold_m");
  check(config.pos_threshold_m, GT, 0.0, "pos_threshold_m");
}

PlaceNodeMatcher::PlaceNodeMatcher(const Config& config)
    : config(config::checkValid(config)) {}

bool PlaceNodeMatcher::match(const Attrs& lhs_attrs, const Attrs& rhs_attrs) const {
  const auto lhs_derived = dynamic_cast<const PlaceNodeAttributes*>(&lhs_attrs);
  const auto rhs_derived = dynamic_cast<const PlaceNodeAttributes*>(&rhs_attrs);
  if (!lhs_derived || !rhs_derived) {
    LOG(WARNING) << "Unable to cast both attributes to PlaceNodeAttributes";
    return false;
  }
  if (!lhs_derived->real_place || !rhs_derived->real_place) {
    return false;
  }

  const auto distance = (lhs_derived->position - rhs_derived->position).norm();
  if (distance > config.pos_threshold_m) {
    return false;
  }

  const auto radii_deviation = std::abs(lhs_derived->distance - rhs_derived->distance);
  return radii_deviation <= config.distance_tolerance_m;
}

void declare_config(PlaceNodeMatcher::Config& config) {
  using namespace config;
  name("PlaceNodeMatcher::Config");
  field(config.pos_threshold_m, "pos_threshold_m");
  field(config.distance_tolerance_m, "distance_tolerance_m");
  check(config.pos_threshold_m, GT, 0.0, "pos_threshold_m");
  check(config.distance_tolerance_m, GT, 0.0, "distance_tolerance_m");
}

}  // namespace hydra
