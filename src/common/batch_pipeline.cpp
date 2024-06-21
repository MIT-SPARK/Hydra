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
#include "hydra/common/batch_pipeline.h"

#include <glog/logging.h>
#include <glog/stl_logging.h>

#include "hydra/backend/update_functions.h"
#include "hydra/backend/update_rooms_buildings_functor.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/reconstruction/mesh_integrator.h"

namespace hydra {

using VFConfig = config::VirtualConfig<FrontendModule>;
using RFConfig = RoomFinderConfig;

BatchPipeline::BatchPipeline(const PipelineConfig& config, int robot_id) {
  GlobalInfo::init(config, robot_id, true);
}

BatchPipeline::~BatchPipeline() {}

DynamicSceneGraph::Ptr BatchPipeline::construct(const VFConfig& frontend_config,
                                                VolumetricMap& map,
                                                const RFConfig* room_config) const {
  if (!map.hasSemantics()) {
    return nullptr;
  }

  MeshIntegratorConfig mesh_config;
  MeshIntegrator integrator(mesh_config);
  integrator.generateMesh(map, false, false);

  auto dsg = GlobalInfo::instance().createSharedDsg();
  auto graph = dsg->graph->clone();
  auto state = std::make_shared<SharedModuleState>();
  auto module = frontend_config.create(dsg, state, LogSetup::Ptr());
  const auto queue = module->getQueue();

  // TODO(nathan) this is a little sketchy given the lack of pose info
  auto msg = std::make_shared<ReconstructionOutput>();
  msg->setMap(map);
  queue->push(msg);

  if (!module->spinOnce()) {
    return nullptr;
  }

  if (room_config) {
    // TODO(nathan) unmerged graph is annoying
    UpdateRoomsFunctor functor(*room_config);
    UpdateInfo::ConstPtr info(new UpdateInfo);
    functor.call(*graph, *dsg, info);

    UpdateBuildingsFunctor bfunctor(Color(), -1);
    bfunctor.call(*graph, *dsg, info);
  }

  return dsg->graph;
}

}  // namespace hydra
