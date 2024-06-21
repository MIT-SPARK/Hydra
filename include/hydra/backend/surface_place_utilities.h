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
#include <glog/logging.h>

#include "hydra/common/dsg_types.h"
#include "hydra/frontend/place_2d_split_logic.h"

namespace hydra::utils {

void getPlace2dAndNeighors(const SceneGraphLayer& places_layer,
                           std::vector<std::pair<NodeId, Place2d>>& place_2ds,
                           std::map<NodeId, std::set<NodeId>>& node_neighbors);

void getNecessaryUpdates(
    const spark_dsg::Mesh& mesh,
    size_t min_points,
    double min_size,
    double connection_ellipse_scale_factor,
    std::vector<std::pair<NodeId, Place2d>>& place_2ds,
    std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
    std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add);

std::map<std::tuple<size_t, size_t, size_t, size_t>, double> buildEdgeMap(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    double place_overlap_threshold,
    double place_neighbor_z_diff);

void updateExistingNodes(const std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
                         DynamicSceneGraph& graph);

NodeSymbol insertNewNodes(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    const double place_overlap_threshold,
    const double place_max_neighbor_z_diff,
    NodeSymbol next_node_symbol,
    DynamicSceneGraph& graph,
    std::map<std::tuple<size_t, size_t>, NodeId>& new_id_map);

void addNewNodeEdges(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>> nodes_to_add,
    const std::map<std::tuple<size_t, size_t, size_t, size_t>, double> edge_map,
    const std::map<std::tuple<size_t, size_t>, NodeId> new_id_map,
    DynamicSceneGraph& graph);

void reallocateMeshPoints(const std::vector<Place2d::PointT>& points,
                          Place2dNodeAttributes& attrs1,
                          Place2dNodeAttributes& attrs2);

}  // namespace hydra::utils
