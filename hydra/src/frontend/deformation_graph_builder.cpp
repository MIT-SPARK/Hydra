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
#include "hydra/frontend/deformation_graph_builder.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo/compression/block_compression.h>
#include <kimera_pgmo/utils/common_functions.h>

#include "hydra/common/global_info.h"
#include "hydra/utils/pgmo_mesh_interface.h"
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;
using Vertices = pcl::PointCloud<pcl::PointXYZRGBA>;
using kimera_pgmo::makePoseGraph;

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilderFunctor,
                                   DeformationGraphBuilder,
                                   DeformationGraphBuilder::Config>(
        "DeformationGraphBuilder");

}

using hydra::timing::ScopedTimer;

void declare_config(DeformationGraphBuilder::Config& config) {
  using namespace config;
  name("DeformationGraphBuilder::Config");
  base<VerbosityConfig>(config);
  field(config.resolution, "resolution", "m");
  field(config.horizon_s, "horizon_s", "s");
  check(config.resolution, GT, 0.0, "resolution");
  check(config.horizon_s, GT, 0.0, "horizon_s");
}

DeformationGraphBuilder::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("dgraph_compression")) {}

DeformationGraphBuilder::DeformationGraphBuilder(const Config& config)
    : config(config::checkValid(config)),
      compression_(new kimera_pgmo::BlockCompression(config.resolution)) {}

DeformationGraphBuilder::~DeformationGraphBuilder() = default;

void DeformationGraphBuilder::call(const ActiveWindowOutput& input,
                                   SharedDsgInfo& dsg,
                                   FrontendOutput& output,
                                   const VolumetricWindow*) {
  using namespace std::chrono;

  ScopedTimer timer("frontend/dgraph_compresssion", input.timestamp_ns, true, 1, false);
  const auto time_ns = nanoseconds(input.timestamp_ns);
  const auto time_s = duration_cast<duration<double>>(time_ns).count();
  compression_->pruneStoredMesh(time_s - config.horizon_s);

  std::vector<size_t> indices;
  std::vector<pcl::Vertices> faces;
  kimera_pgmo::HashedIndexMapping remapping;
  pcl::PointCloud<pcl::PointXYZRGBA> points;
  auto mesh = PgmoMeshLayerInterface(input.map().getMeshLayer());
  compression_->compressAndIntegrate(mesh, points, faces, indices, remapping, time_s);

  Vertices::Ptr vertices(new Vertices());
  compression_->getVertices(vertices);

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto edges = graph_.addPointsAndSurfaces(indices, faces);
  auto new_graph = makePoseGraph(prefix.id, time_s, edges, indices, *vertices);

  // move deformation graph to output
  std::lock_guard<std::mutex> lock(dsg.mutex);
  output.deformation_graph = new_graph;
}

}  // namespace hydra
