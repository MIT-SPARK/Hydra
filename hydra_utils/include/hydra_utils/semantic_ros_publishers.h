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
