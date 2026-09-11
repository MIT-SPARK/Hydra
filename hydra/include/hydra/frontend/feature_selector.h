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

struct FeatureSelector {
  using FeatureList = std::vector<FeatureView>;
  struct Config {
    //! Max range beyond range image
    double max_range_difference_m = 0.1;
  };

  virtual ~FeatureSelector() = default;
  virtual bool select(const FeatureList& views,
                      spark_dsg::SemanticNodeAttributes& attrs) const = 0;
};

void declare_config(FeatureSelector::Config& config);

//! Choses the feature of the view closest to the place that the place is visible from
struct ClosestFeatureSelector : FeatureSelector {
  struct Config : FeatureSelector::Config {
  } const config;

  explicit ClosestFeatureSelector(const Config& config);
  bool select(const FeatureList& views,
              spark_dsg::SemanticNodeAttributes& attrs) const override;
};

void declare_config(ClosestFeatureSelector::Config& config);

//! Averages the features of all views the place is visible from
struct AverageFeatureSelector : FeatureSelector {
  struct Config : FeatureSelector::Config {
  } const config;

  explicit AverageFeatureSelector(const Config& config);
  bool select(const FeatureList& views,
              spark_dsg::SemanticNodeAttributes& attrs) const override;
};

void declare_config(AverageFeatureSelector::Config& config);

//! Fits a von-mises fischer distribution to features of views the place is visible from
struct VMFFeatureSelector : FeatureSelector {
  struct Config : FeatureSelector::Config {
    float kappa_max = 500.0f;
  } const config;

  explicit VMFFeatureSelector(const Config& config);
  bool select(const FeatureList& views,
              spark_dsg::SemanticNodeAttributes& attrs) const override;
};

void declare_config(VMFFeatureSelector::Config& config);

}  // namespace hydra
