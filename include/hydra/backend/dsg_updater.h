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
#include <config_utilities/virtual_config.h>
#include <kimera_pgmo/kimera_pgmo_interface.h>
#include <spark_dsg/scene_graph_logger.h>

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra/backend/backend_input.h"
#include "hydra/backend/external_loop_closure_receiver.h"
#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/pgmo_configs.h"
#include "hydra/common/output_sink.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/utils/data_directory.h"

namespace hydra {

class DsgUpdater {
 public:
  using Ptr = std::shared_ptr<DsgUpdater>;
  using Sink = OutputSink<uint64_t,
                          const DynamicSceneGraph&,
                          const kimera_pgmo::DeformationGraph&>;
  using NodeToRobotMap = std::unordered_map<NodeId, size_t>;

  struct Config {
    //! Enable combining multiple nodes together
    bool enable_node_merging = true;
    //! Repeatedly run merge detection until no more merges are detected
    bool enable_exhaustive_merging = false;
    //! Update functors that get applied in the specified order
    config::OrderedMap<std::string, config::VirtualConfig<UpdateFunctor, true>>
        update_functors;
  } const config;

  DsgUpdater(const Config& config,
             DynamicSceneGraph::Ptr source,
             SharedDsgInfo::Ptr target);

  virtual ~DsgUpdater() = default;

  DsgUpdater(const DsgUpdater& other) = delete;

  DsgUpdater& operator=(const DsgUpdater& other) = delete;

  void save(const DataDirectory& output, const std::string& label) const;

  void resetBackendDsg(size_t timestamp_ns);

  void callUpdateFunctions(size_t timestamp_ns, UpdateInfo::ConstPtr info);

 private:
  MergeTracker merge_tracker;
  std::map<std::string, UpdateFunctor::Ptr> update_functors_;

  DynamicSceneGraph::Ptr source_graph_;
  SharedDsgInfo::Ptr target_dsg_;
};

void declare_config(DsgUpdater::Config& conf);

}  // namespace hydra
