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
#include "hydra_topology/topology_server.h"

#include <hydra_utils/timing_utilities.h>
#include <kimera_semantics_ros/semantic_tsdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include <glog/logging.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>

using hydra::timing::ElapsedTimeRecorder;
namespace fs = boost::filesystem;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "hydra_topology_node");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle pnh("~");

  std::string output_path = "";
  pnh.getParam("log_path", output_path);

  fs::path timer_path(output_path);
  timer_path /= "topology";

  if (output_path != "") {
    boost::system::error_code code;
    if (!fs::create_directory(timer_path, code)) {
      ROS_WARN_STREAM("Failed to create: " << timer_path);
    }
  }

  bool log_timing_incrementally = false;
  pnh.getParam("log_timing_incrementally", log_timing_incrementally);
  if (log_timing_incrementally && output_path != "") {
    ElapsedTimeRecorder::instance().setupIncrementalLogging(output_path);
  }

  bool use_semantic_tsdf_server = false;
  pnh.getParam("use_semantic_tsdf_server", use_semantic_tsdf_server);
  if (use_semantic_tsdf_server) {
    ROS_DEBUG("Using Semantic TSDF Server");
    hydra::topology::TopologyServer<kimera::SemanticTsdfServer> server(pnh);
    server.spin();
  } else {
    ROS_DEBUG("Using Normal (Non-Semantic) TSDF Server");
    hydra::topology::TopologyServer<voxblox::TsdfServer> server(pnh);
    server.spin();
  }

  if (output_path != "") {
    const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
    timer.logAllElapsed(output_path);
    timer.logStats(timer_path.native());
  }

  return 0;
}
