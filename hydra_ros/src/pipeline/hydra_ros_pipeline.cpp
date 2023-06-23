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
#include "hydra_ros/pipeline/hydra_ros_pipeline.h"

#include "hydra_ros/config/ros_utilities.h"
#include "hydra_ros/pipeline/ros_lcd_registration.h"

namespace hydra {

template <typename Visitor>
void visit_config(const Visitor& v, HydraRosConfig& config) {
  v.visit("enable_lcd", config.enable_lcd);
  v.visit("use_ros_backend", config.use_ros_backend);
  v.visit("do_reconstruction", config.do_reconstruction);
  v.visit("enable_frontend_output", config.enable_frontend_output);
  v.visit("frontend_mesh_separation_s", config.frontend_mesh_separation_s);
}

DECLARE_STRUCT_NAME(HydraRosConfig);
DECLARE_STRUCT_NAME(FrontendConfig);
DECLARE_STRUCT_NAME(BackendConfig);
DECLARE_STRUCT_NAME(LoopClosureConfig);
DECLARE_STRUCT_NAME(kimera_pgmo::KimeraPgmoConfig);

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, HydraRosConfig)

namespace hydra {

HydraRosPipeline::HydraRosPipeline(const ros::NodeHandle& node_handle, int robot_id)
    : nh(node_handle), prefix(robot_id) {
  config = load_config<hydra::HydraRosConfig>(nh);

  // TODO(nathan) parse and use at some point
  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char> layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                             {DsgLayers::PLACES, 'p'},
                                             {DsgLayers::ROOMS, 'r'},
                                             {DsgLayers::BUILDINGS, 'b'}};

  frontend_dsg.reset(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  backend_dsg.reset(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  shared_state.reset(new SharedModuleState());

  if (config.do_reconstruction) {
    const auto frontend_config = load_config<FrontendConfig>(nh);
    frontend = std::make_shared<FrontendModule>(
        prefix, frontend_config, frontend_dsg, shared_state);
    reconstruction =
        std::make_shared<RosReconstruction>(nh, prefix, frontend->getQueue());
  } else {
    frontend = std::make_shared<RosFrontend>(nh, prefix, frontend_dsg, shared_state);
  }

  if (config.enable_frontend_output) {
    ros::NodeHandle frontend_nh(nh, "frontend");
    dsg_sender.reset(new DsgSender(
        frontend_nh, "frontend", true, config.frontend_mesh_separation_s));
    mesh_graph_pub =
        nh.advertise<pose_graph_tools::PoseGraph>("mesh_graph_incremental", 100, true);
    mesh_update_pub =
        nh.advertise<kimera_pgmo::KimeraPgmoMeshDelta>("full_mesh_update", 100, true);
    frontend->addOutputCallback(std::bind(&HydraRosPipeline::sendFrontendOutput,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          std::placeholders::_3));
  }

  const auto backend_config = load_config<BackendConfig>(nh);
  if (config.use_ros_backend) {
    backend = std::make_shared<RosBackend>(
        nh, prefix, frontend_dsg, backend_dsg, shared_state);
  } else {
    const auto pgmo_config = load_config<kimera_pgmo::KimeraPgmoConfig>(nh, "pgmo");
    backend = std::make_shared<BackendModule>(
        prefix, backend_config, pgmo_config, frontend_dsg, backend_dsg, shared_state);
  }

  backend_visualizer =
      std::make_shared<RosBackendVisualizer>(nh, backend_config, prefix);
  backend->addOutputCallback(std::bind(&RosBackendVisualizer::publishOutputs,
                                       backend_visualizer.get(),
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3));

  if (config.enable_lcd) {
    auto lcd_config = load_config<LoopClosureConfig>(nh, "");
    lcd_config.detector.num_semantic_classes = frontend->maxSemanticLabel();
    VLOG(1) << "Number of classes for LCD: "
            << lcd_config.detector.num_semantic_classes;
    shared_state->lcd_queue.reset(new InputQueue<LcdInput::Ptr>());
    lcd.reset(new LoopClosureModule(prefix, lcd_config, frontend_dsg, shared_state));

    bow_sub = nh.subscribe("bow_vectors", 100, &HydraRosPipeline::bowCallback, this);
    if (lcd_config.detector.enable_agent_registration) {
      lcd->getDetector().setRegistrationSolver(0, std::make_unique<DsgAgentSolver>());
    }
  }
}

void HydraRosPipeline::bowCallback(const pose_graph_tools::BowQueries::ConstPtr& msg) {
  for (const auto& query : msg->queries) {
    shared_state->visual_lcd_queue.push(
        pose_graph_tools::BowQuery::ConstPtr(new pose_graph_tools::BowQuery(query)));
  }
}

void HydraRosPipeline::start() {
  if (reconstruction) {
    reconstruction->start();
  }

  if (frontend) {
    frontend->start();
  }

  if (backend) {
    backend->start();
  }

  if (lcd) {
    lcd->start();
  }
}

void HydraRosPipeline::stop() {
  if (reconstruction) {
    reconstruction->stop();
  }

  if (frontend) {
    frontend->stop();
  }

  if (lcd) {
    lcd->stop();
  }

  if (backend) {
    backend->stop();
  }
}

void HydraRosPipeline::save(const std::string& output_path) {
  if (!output_path.empty()) {
    if (reconstruction) {
      reconstruction->save(output_path + "/topology/");
    }

    if (frontend) {
      frontend->save(output_path + "/frontend/");
    }

    if (backend) {
      backend->save(output_path + "/backend/");
    }

    if (lcd) {
      lcd->save(output_path + "/lcd/");
    }
  }
}

void HydraRosPipeline::sendFrontendOutput(const DynamicSceneGraph& graph,
                                          const BackendInput& backend_input,
                                          uint64_t timestamp_ns) {
  if (backend_input.deformation_graph) {
    mesh_graph_pub.publish(*backend_input.deformation_graph);
  }

  if (backend_input.mesh_update) {
    mesh_update_pub.publish(backend_input.mesh_update->toRosMsg(timestamp_ns));
  }

  sendFrontendGraph(graph, timestamp_ns);
}

void HydraRosPipeline::sendFrontendGraph(const DynamicSceneGraph& graph,
                                         uint64_t timestamp_ns) {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);
  dsg_sender->sendGraph(graph, stamp);
}

}  // namespace hydra
