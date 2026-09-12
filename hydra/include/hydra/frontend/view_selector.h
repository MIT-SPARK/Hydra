#pragma once
#include <spark_dsg/node_attributes.h>

#include <opencv2/core/mat.hpp>

#include "hydra/openset/openset_types.h"

namespace hydra {

class Sensor;
struct InputData;

struct FeatureView {
  using Ptr = std::unique_ptr<FeatureView>;
  explicit FeatureView(const InputData& data);

  const uint64_t timestamp_ns;
  const Sensor& sensor;
  const FeatureVector& feature;
  const cv::Mat range_image;
  const Eigen::Isometry3d sensor_T_world;

  bool pointInView(const Eigen::Vector3d& point_w,
                   float max_range_difference_m,
                   Eigen::Vector3d* point_s = nullptr) const;
};

struct ViewSelector {
  using FeatureList = std::vector<FeatureView>;

  virtual ~ViewSelector() = default;
  virtual bool selectFeature(const FeatureList& views,
                             float max_range_difference_m,
                             spark_dsg::SemanticNodeAttributes& attrs) const = 0;
};

struct ClosestViewSelector : ViewSelector {
  bool selectFeature(const FeatureList& views,
                     float max_range_difference_m,
                     spark_dsg::SemanticNodeAttributes& attrs) const override;
};

struct AverageViewSelector : ViewSelector {
  bool selectFeature(const FeatureList& views,
                     float max_range_difference_m,
                     spark_dsg::SemanticNodeAttributes& attrs) const override;
};

}  // namespace hydra
