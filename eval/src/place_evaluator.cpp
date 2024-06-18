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
#include "hydra/eval/place_evaluator.h"

#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <glog/logging.h>

#include "hydra/places/gvd_integrator.h"
#include "hydra/reconstruction/volumetric_map.h"
#include "hydra/utils/layer_io.h"

namespace hydra::eval {

using places::GvdIntegratorConfig;
using places::GvdLayer;
using places::GvdVoxel;

PlaceEvaluator::PlaceEvaluator(const GvdIntegratorConfig& config,
                               const TsdfLayer::Ptr& tsdf)
    : tsdf_(CHECK_NOTNULL(tsdf)) {
  gvd_.reset(new GvdLayer(tsdf_->voxel_size, tsdf_->voxels_per_side));
  computeGroundTruth(config);
}

void PlaceEvaluator::computeGroundTruth(const GvdIntegratorConfig& config) {
  config_ = config;

  VLOG(1) << "using GVD config:" << std::endl << config_;
  places::GvdIntegrator integrator(config_, gvd_, nullptr);
  auto map = VolumetricMap::fromTsdf(*tsdf_, 0.3, false);
  CHECK(map) << "Invalid map!";
  integrator.updateFromTsdf(0, map->getTsdfLayer(), false, true);
  integrator.updateGvd(0);
}

PlaceEvaluator::Ptr PlaceEvaluator::fromFile(const std::string& config_filepath,
                                             const std::string& tsdf_filepath,
                                             std::optional<double> max_distance_m) {
  auto config = config::fromYamlFile<GvdIntegratorConfig>(config_filepath);
  if (max_distance_m) {
    config.max_distance_m = *max_distance_m;
  }

  auto tsdf = io::loadLayer<TsdfLayer>(tsdf_filepath);
  if (!tsdf) {
    LOG(ERROR) << "Failed to load TSDF from: " << tsdf_filepath;
    return nullptr;
  }

  return std::make_unique<PlaceEvaluator>(config, tsdf);
}

PlaceMetrics PlaceEvaluator::eval(const std::string& graph_filepath) const {
  const auto graph = DynamicSceneGraph::load(graph_filepath);
  if (!graph->hasLayer(DsgLayers::PLACES)) {
    LOG(ERROR) << "Graph file: " << graph_filepath << " does not have places";
    return {};
  }

  const auto& places = graph->getLayer(DsgLayers::PLACES);
  LOG(INFO) << "Place Nodes: " << places.nodes().size();
  return scorePlaces(places, *gvd_, config_.min_basis_for_extraction);
}

}  // namespace hydra::eval
