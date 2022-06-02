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
// originally part of kimera_batch_dsg
#include <ros/ros.h>

#include <string>
#include <unordered_map>

namespace hydra {

template <class LabelType, class MsgType>
class SemanticRosPublishers {
  typedef std::unordered_map<LabelType, ros::Publisher> SemanticRosPubs;

 public:
  /**
   * @brief SemanticRosPublishers Publishes a msg in a different ROS topic
   * defined by the semantic label. This is a convenience class
   * to allow for publishing msgs of the same type in different ROS topics.
   * @param topic_name String to be prepended to all the ROS topics advertised.
   * @param nh_private Node handle to create the ROS publishers.
   */
  SemanticRosPublishers(const std::string& topic_name,
                        const ros::NodeHandle& nh_private)
      : topic_name_(topic_name), nh_private_(nh_private), pubs_() {}
  virtual ~SemanticRosPublishers() = default;

  /** publish Publishes a given ROS msg for a given semantic_label under the
   * same ROS topic.
   * If the given semantic_label has not been published before, a new ROS topic
   * is advertised with the name given by topic_name_.
   */
  void publish(const LabelType& semantic_label, const MsgType& msg) {
    // Publish centroids for each semantic label
    const auto& pub_it = pubs_.find(semantic_label);
    if (pub_it == pubs_.end()) {
      pubs_[semantic_label] = nh_private_.advertise<MsgType>(
          topic_name_ + "_semantic_label_" + std::to_string(semantic_label), 1, true);
    }
    // Publish semantic pointcloud
    pubs_.at(semantic_label).publish(msg);
  }

  /**
   * Derive the topic name from the label.
   */
  std::string getTopic(const LabelType& semantic_label) {
    const auto& pub_it = pubs_.find(semantic_label);
    if (pub_it == pubs_.end()) {
      pubs_[semantic_label] = nh_private_.advertise<MsgType>(
          topic_name_ + "_semantic_label_" + std::to_string(semantic_label), 1, true);
    }
    return pubs_.at(semantic_label).getTopic();
  }

  std::string get_prefix() { return topic_name_; }

 private:
  /// Prepended string to the ROS topic that is to be advertised.
  std::string topic_name_;

  ros::NodeHandle nh_private_;
  SemanticRosPubs pubs_;
};

}  // namespace hydra
