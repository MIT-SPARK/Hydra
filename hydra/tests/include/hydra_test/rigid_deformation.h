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

#include <gtsam/inference/Symbol.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

#include "hydra_test/temp_directory.h"

namespace hydra::test {

/**
 * @brief Deformation graph whose control points all move by the same rigid
 * transform, so that every interpolated node transform is exactly that transform.
 *
 * @param world_T_world Transform applied to every control point.
 * @param robot_id Robot whose mesh vertex prefix the control points use.
 * @param stamp_ns Timestamp of every control point.
 */
inline kimera_pgmo::DeformationGraph::Ptr makeRigidDeformationGraph(
    const Eigen::Isometry3d& world_T_world,
    size_t robot_id = 0,
    uint64_t stamp_ns = 10) {
  const auto prefix = kimera_pgmo::GetVertexPrefix(robot_id);
  const Eigen::Quaterniond q(world_T_world.linear());
  // start from the serialized form of an empty graph so every field kimera_pgmo expects
  // is present, then add the control points
  const TempDirectory tmp("rigid_deformation");
  const auto path = tmp.path / "graph.json";
  kimera_pgmo::DeformationGraph().save(path.string());
  nlohmann::json root;
  {
    std::ifstream in(path);
    root = nlohmann::json::parse(in);
  }

  // control points use the "vertices" layout, which kimera_pgmo reads when the graph
  // has no "control_points" entry
  root.erase("control_points");
  const std::string prefix_str(1, prefix);
  auto& vertices = root["vertices"][prefix_str];
  vertices["pos"] = nlohmann::json::array();
  vertices["stamps"] = nlohmann::json::array();
  root["values"] = nlohmann::json::array();
  // control points on a small grid so any query has enough neighbors
  size_t index = 0;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      const Eigen::Vector3d point(x, y, 0.0);
      // the interpolated transform is R * (p - c) + t with t = R * c + translation
      const Eigen::Vector3d t = world_T_world * point;
      root["values"].push_back({{"key", gtsam::Symbol(prefix, index).key()},
                                {"value",
                                 {{"x", t.x()},
                                  {"y", t.y()},
                                  {"z", t.z()},
                                  {"qw", q.w()},
                                  {"qx", q.x()},
                                  {"qy", q.y()},
                                  {"qz", q.z()}}}});
      vertices["pos"].push_back({point.x(), point.y(), point.z()});
      vertices["stamps"].push_back(stamp_ns);
      ++index;
    }
  }

  {
    std::ofstream out(path);
    out << root;
  }

  return kimera_pgmo::DeformationGraph::load(path);
}

}  // namespace hydra::test
