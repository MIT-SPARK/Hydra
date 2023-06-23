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
#include <ros/ros.h>

#include <iomanip>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

struct SceneGraphLoggerNode {
  SceneGraphLoggerNode(const ros::NodeHandle& nh)
      : nh_(nh), curr_count_(0), curr_output_count_(0), output_every_num_(1) {
    receiver_.reset(new DsgReceiver(nh_));
    if (!nh_.getParam("output_path", output_path_)) {
      ROS_FATAL("Failed to get output path parameter");
      throw std::runtime_error("failed to get output path");
    }

    nh_.getParam("output_every_num", output_every_num_);
  }

  void spin() {
    ros::Rate r(10);
    while (ros::ok()) {
      ros::spinOnce();

      if (!receiver_->updated()) {
        r.sleep();
        continue;
      }

      receiver_->clearUpdated();
      ++curr_count_;
      if (output_every_num_ == 1 || curr_count_ % output_every_num_ != 1) {
        continue;
      }

      std::stringstream ss;
      ss << output_path_ << "/dsg_" << std::setfill('0') << std::setw(6)
         << curr_output_count_;
      ss << ".json";
      receiver_->graph()->save(ss.str(), false);
      ++curr_output_count_;
    }
  }

  ros::NodeHandle nh_;
  size_t curr_count_;
  size_t curr_output_count_;
  int output_every_num_;
  std::string output_path_;
  std::unique_ptr<DsgReceiver> receiver_;
};

}  // namespace hydra

int main(int argc, char** argv) {
  ros::init(argc, argv, "scene_graph_logger_node");

  ros::NodeHandle nh("~");
  hydra::SceneGraphLoggerNode node(nh);
  node.spin();

  return 0;
}
