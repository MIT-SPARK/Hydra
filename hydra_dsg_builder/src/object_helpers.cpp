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
#include "hydra_dsg_builder/object_helpers.h"

#include <glog/logging.h>
#include <pcl/common/centroid.h>
#include <spark_dsg/bounding_box_extraction.h>

namespace hydra {

using Centroid = pcl::CentroidPoint<pcl::PointXYZ>;

bool updateObjectSpatialAttributes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                                   const pcl::IndicesPtr& indices,
                                   ObjectNodeAttributes& attrs) {
  attrs.bounding_box = bounding_box::extract(cloud, attrs.bounding_box.type, indices);

  Centroid centroid;
  for (const auto& idx : *indices) {
    const auto& point = cloud->at(idx);
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      VLOG(4) << "found nan at index: " << idx << " with point: [" << point.x << ", "
              << point.y << ", " << point.z << "]";
      continue;
    }

    centroid.add(pcl::PointXYZ(point.x, point.y, point.z));
  }

  if (!centroid.getSize()) {
    return false;
  }

  pcl::PointXYZ pcl_pos;
  centroid.get(pcl_pos);
  attrs.position << pcl_pos.x, pcl_pos.y, pcl_pos.z;
  return true;
}

}  // namespace hydra
