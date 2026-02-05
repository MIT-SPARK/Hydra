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
#include <spark_dsg/node_attributes.h>

namespace hydra {

struct NodeMatcher {
  using Attrs = spark_dsg::NodeAttributes;

  virtual ~NodeMatcher() = default;
  virtual bool match(const Attrs& new_attributes,
                     const Attrs& prev_attributes) const = 0;
};

struct CentroidBBoxMatcher : public NodeMatcher {
  struct Config {};

  explicit CentroidBBoxMatcher(const Config&) {}
  bool match(const Attrs& new_attrs, const Attrs& prev_attrs) const override;
};

void declare_config(CentroidBBoxMatcher::Config& config);

struct IoUNodeMatcher : public NodeMatcher {
  struct Config {
    //! @brief Minimum IoU between two nodes of same class
    double min_same_iou = 0.2;
    //! @brief Minimum IoU between two nodes of different classes
    double min_cross_iou = 0.5;
  } const config;

  explicit IoUNodeMatcher(const Config& config);
  bool match(const Attrs& new_attrs, const Attrs& prev_attrs) const override;
};

void declare_config(IoUNodeMatcher::Config& config);

struct DistanceNodeMatcher : public NodeMatcher {
  struct Config {
    //! Max distance between node centroids for a merge to be considered
    double pos_threshold_m = 0.4;
  } const config;

  explicit DistanceNodeMatcher(const Config& config);
  bool match(const Attrs& new_attrs, const Attrs& prev_attrs) const override;
};

void declare_config(DistanceNodeMatcher::Config& config);

struct PlaceNodeMatcher : public NodeMatcher {
  struct Config {
    //! Max distance between node centroids for a merge to be considered
    double pos_threshold_m = 0.4;
    //! Max deviation between place radii for a merge to be considered
    double distance_tolerance_m = 0.4;
  } const config;

  explicit PlaceNodeMatcher(const Config& config);
  bool match(const Attrs& new_attrs, const Attrs& prev_attrs) const override;
};

void declare_config(PlaceNodeMatcher::Config& config);

}  // namespace hydra
