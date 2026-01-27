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
#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>
#include <hydra/backend/dsg_updater.h>
#include <hydra/backend/update_functions.h>
#include <hydra/common/global_info.h>
#include <hydra/common/shared_dsg_info.h>
#include <hydra/utils/data_directory.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include "hydra/common/shared_dsg_info.h"

#include <iostream>

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cerr << "missing dsg file and/or config and/or output directory! usage: object_reconciliation dsg_file config_file output_dir"
              << std::endl;
    return 1;
  }

  // Load in scene graph
  const auto graph = spark_dsg::DynamicSceneGraph::load(argv[1]);

  // SharedDsgInfo::Ptr private_dsg_;
  const hydra::PipelineConfig global_config;
  hydra::GlobalInfo::init(global_config, 0);

  hydra::SharedDsgInfo::Ptr private_dsg_ = hydra::GlobalInfo::instance().createSharedDsg();
  private_dsg_->graph = graph;


  
  std::cout << "Loaded DSG with " << private_dsg_->graph->numNodes() << " nodes" << std::endl;

  // Load configuration from YAML file
  const auto config = config::fromYamlFile<hydra::DsgUpdater::Config>(argv[2]);

  auto output_path = std::filesystem::path(argv[3]);
  if (!std::filesystem::exists(output_path)) {
    std::filesystem::create_directories(output_path);
  }
  
  // Create shared DSG info for backend
  hydra::DynamicSceneGraph::Ptr unmerged_graph_ = private_dsg_->graph->clone();
  
  // Initialize dsg updater
  hydra::DsgUpdater::Ptr dsg_updater;
  dsg_updater.reset(new hydra::DsgUpdater(config, unmerged_graph_, private_dsg_)); 
  
  // Print how many objects are in the scene graph
  const auto& objects_layer = unmerged_graph_->getLayer(spark_dsg::DsgLayers::OBJECTS);
  std::cout << "Frontend DSG has " << objects_layer.numNodes() << " objects" << std::endl;
  
  // Call dsg updater callUpdateFunctions
  hydra::UpdateInfo::ConstPtr update_info(new hydra::UpdateInfo{0});
  dsg_updater->callUpdateFunctions(0, update_info);
  
  // Call dsg updater save
  const hydra::DataDirectory output_dir(output_path);
  dsg_updater->save(output_dir, "reconciled");
  
  // Print how many objects are in the updated scene graph
  const auto& backend_objects_layer = private_dsg_->graph->getLayer(spark_dsg::DsgLayers::OBJECTS);
  std::cout << "Merged DSG has " << backend_objects_layer.numNodes() << " objects" << std::endl;
  
  
  std::cout << "Object reconciliation complete! Output saved to " << output_path << std::endl;
  return 0;
}