#include "hydra/frontend/feature_selector.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

#include "hydra/input/input_data.h"
#include "hydra/openset/vmf_distance.h"

namespace hydra {
namespace {

static const auto closest_reg =
    config::RegistrationWithConfig<FeatureSelector,
                                   ClosestFeatureSelector,
                                   ClosestFeatureSelector::Config>("closest");

static const auto average_reg =
    config::RegistrationWithConfig<FeatureSelector,
                                   AverageFeatureSelector,
                                   AverageFeatureSelector::Config>("average");

static const auto vmf_reg =
    config::RegistrationWithConfig<FeatureSelector,
                                   VMFFeatureSelector,
                                   VMFFeatureSelector::Config>("vmf");

}  // namespace

using spark_dsg::SemanticNodeAttributes;

FeatureView::FeatureView(const InputData& data)
    : timestamp_ns(data.timestamp_ns),
      sensor(data.getSensor()),
      feature(data.feature),
      range_image(data.range_image),
      sensor_T_world(data.getSensorPose().inverse()) {}

bool FeatureView::pointInView(const Eigen::Vector3d& point_w,
                              float max_range_difference_m,
                              Eigen::Vector3d* point_s) const {
  const Eigen::Vector3d p_s = (sensor_T_world * point_w);
  if (point_s) {
    *point_s = p_s;
  }

  float u = 0.0;
  float v = 0.0;
  if (!sensor.projectPointToImagePlane(p_s.cast<float>(), u, v)) {
    return false;
  }

  const auto r = std::round(u);
  const auto c = std::round(v);
  if (c < 0 || c >= range_image.cols || r < 0 || r >= range_image.rows) {
    return false;
  }

  const auto range = range_image.at<InputData::RangeType>(v, u);
  if (range < 1.0e-6f || !std::isfinite(range)) {
    return false;
  }

  // This is positive if the point is occluded by the observed range image
  const auto diff = p_s.norm() - range;
  return diff < max_range_difference_m;
}

void declare_config(FeatureSelector::Config& config) {
  using namespace config;
  name("FeatureSelector::Config");
  field(config.max_range_difference_m, "max_range_difference_m");
  check(config.max_range_difference_m, GT, 0.0, "max_range_difference_m");
}

void declare_config(ClosestFeatureSelector::Config& config) {
  using namespace config;
  name("ClosestFeatureSelector::Config");
  base<FeatureSelector::Config>(config);
}

ClosestFeatureSelector::ClosestFeatureSelector(const Config& config)
    : config(config::checkValid(config)) {}

bool ClosestFeatureSelector::select(const FeatureList& views,
                                    SemanticNodeAttributes& attrs) const {
  const FeatureView* best_view = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& view : views) {
    if (view.feature.size() == 0) {
      continue;
    }

    Eigen::Vector3d p_s;
    if (!view.pointInView(attrs.position, config.max_range_difference_m, &p_s)) {
      continue;
    }

    // norm of position in sensor frame is distance in world frame
    const auto dist = p_s.norm();
    if (dist < min_dist) {
      best_view = &view;
      min_dist = dist;
    }
  }

  if (best_view) {
    attrs.semantic_feature = best_view->feature;
  }

  return best_view != nullptr;
}

void declare_config(AverageFeatureSelector::Config& config) {
  using namespace config;
  name("AverageFeatureSelector::Config");
  base<FeatureSelector::Config>(config);
}

AverageFeatureSelector::AverageFeatureSelector(const Config& config)
    : config(config::checkValid(config)) {}

bool AverageFeatureSelector::select(const FeatureList& views,
                                    SemanticNodeAttributes& attrs) const {
  size_t num_visible = 0;
  for (const auto& view : views) {
    if (view.feature.size() == 0) {
      continue;
    }

    if (!view.pointInView(attrs.position, config.max_range_difference_m)) {
      continue;
    }

    if (!num_visible) {
      attrs.semantic_feature = view.feature;
    } else {
      attrs.semantic_feature += view.feature;
    }
    ++num_visible;
  }

  if (num_visible > 0) {
    attrs.semantic_feature /= num_visible;
  }

  return num_visible > 0;
}

void declare_config(VMFFeatureSelector::Config& config) {
  using namespace config;
  name("VMFFeatureSelector::Config");
  base<FeatureSelector::Config>(config);
  field(config.kappa_max, "kappa_max");
  check(config.kappa_max, GT, 0.0, "kappa_max");
}

VMFFeatureSelector::VMFFeatureSelector(const Config& config)
    : config(config::checkValid(config)) {}

bool VMFFeatureSelector::select(const FeatureList& views,
                                SemanticNodeAttributes& attrs) const {
  size_t num_visible = 0;
  for (const auto& view : views) {
    if (view.feature.size() == 0) {
      continue;
    }

    if (!view.pointInView(attrs.position, config.max_range_difference_m)) {
      continue;
    }

    if (!num_visible) {
      attrs.semantic_feature = view.feature.normalized();
    } else {
      attrs.semantic_feature += view.feature.normalized();
    }
    ++num_visible;
  }

  if (num_visible > 0) {
    const auto stats = computeVmfStats(attrs.semantic_feature, num_visible);
    attrs.semantic_feature = stats.mu;
    // TODO(nathan) need to think about whether or not to combine concentration and
    // feature or to add new attribute field
  }

  return num_visible > 0;
}

}  // namespace hydra
