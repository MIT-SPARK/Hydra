#include "hydra/frontend/view_selector.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>

#include "hydra/input/input_data.h"

namespace hydra {
namespace {

static const auto closest_reg =
    config::Registration<ViewSelector, ClosestViewSelector>("closest");

static const auto average_reg =
    config::Registration<ViewSelector, AverageViewSelector>("average");

}  // namespace

using spark_dsg::SemanticNodeAttributes;

FeatureView::FeatureView(const InputData& data)
    : sensor(data.getSensor()),
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

bool ClosestViewSelector::selectFeature(const FeatureList& views,
                                        float max_range_difference_m,
                                        SemanticNodeAttributes& attrs) const {
  const FeatureView* best_view = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& view : views) {
    if (view.feature.size() == 0) {
      continue;
    }

    Eigen::Vector3d p_s;
    if (!view.pointInView(attrs.position, max_range_difference_m, &p_s)) {
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

bool AverageViewSelector::selectFeature(const FeatureList& views,
                                        float max_range_difference_m,
                                        SemanticNodeAttributes& attrs) const {
  size_t num_visible = 0;
  for (const auto& view : views) {
    if (view.feature.size() == 0) {
      continue;
    }

    if (!view.pointInView(attrs.position, max_range_difference_m)) {
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

}  // namespace hydra
