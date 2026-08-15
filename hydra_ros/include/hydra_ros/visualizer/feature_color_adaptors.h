#pragma once
#include <hydra/openset/embedding_distances.h>
#include <hydra_visualizer/adapters/node_color.h>
#include <hydra_visualizer/adapters/text.h>

#include <rclcpp/subscription.hpp>
#include <semantic_inference_msgs/msg/feature_vector_stamped.hpp>

#include "hydra_ros/openset/ros_embedding_group.h"

namespace hydra {

class FeatureScoreColor : public NodeColorAdapter {
 public:
  struct Config {
    std::string ns = "~";
    float min_score = 0.0f;
    float max_score = 1.0f;
    bool use_fixed_range = false;
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
  } const config;

  explicit FeatureScoreColor(const Config& config);
  ~FeatureScoreColor();
  void setGraph(const spark_dsg::DynamicSceneGraph& graph,
                spark_dsg::LayerKey layer) override;
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

  void setFeature(const Eigen::VectorXf& feature);

 private:
  using InputMsg = semantic_inference_msgs::msg::FeatureVectorStamped;

  void callback(const InputMsg::ConstSharedPtr& msg);

  rclcpp::Subscription<InputMsg>::SharedPtr sub_;

  struct ScoreRange {
    float min = 0.0f;
    float max = 1.0f;
  } range_;

  bool has_change_;
  bool has_feature_;
  Eigen::VectorXf feature_;
  std::unordered_map<spark_dsg::NodeId, float> values_;
  std::unique_ptr<EmbeddingDistance> metric_;
};

void declare_config(FeatureScoreColor::Config& config);

class NearestFeatureColor : public NodeColorAdapter {
 public:
  struct Config {
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
    config::VirtualConfig<EmbeddingGroup> features{RosEmbeddingGroup::Config()};
    visualizer::DiscreteColormap::Config colormap;
  } const config;

  explicit NearestFeatureColor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  std::unique_ptr<EmbeddingDistance> metric_;
  std::unique_ptr<EmbeddingGroup> features_;
  const visualizer::DiscreteColormap colormap_;
};

void declare_config(NearestFeatureColor::Config& config);

class NearestFeatureLabel : public visualizer::NodeTextAdapter {
 public:
  struct Config {
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
    config::VirtualConfig<EmbeddingGroup> features{RosEmbeddingGroup::Config()};
    //! Breaks label around spaces to meet max width if > 0
    size_t label_width = 0;
  } const config;

  explicit NearestFeatureLabel(const Config& config);
  std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const override;

 private:
  std::unique_ptr<EmbeddingDistance> metric_;
  std::unique_ptr<EmbeddingGroup> features_;
};

}  // namespace hydra
