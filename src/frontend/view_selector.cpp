#include "hydra/frontend/view_selector.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>

#include "hydra/input/sensor.h"

namespace hydra {

using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SemanticNodeAttributes;

FeatureView::FeatureView(uint64_t timestamp_ns,
                         const Eigen::Isometry3d& sensor_T_world,
                         const FeatureVector& feature,
                         const Sensor* sensor)
    : timestamp_ns(timestamp_ns),
      sensor_T_world(sensor_T_world),
      feature(feature),
      sensor_(sensor) {
  CHECK(sensor_);
}

const Sensor& FeatureView::sensor() const { return *sensor_; }

bool FeatureView::pointInView(const Eigen::Vector3d& point_w,
                              Eigen::Vector3d* point_s) const {
  const Eigen::Vector3d p_s = (sensor_T_world * point_w);
  if (point_s) {
    *point_s = p_s;
  }

  return sensor_->pointIsInViewFrustum(p_s.cast<float>());
}

struct BoundaryViewSelector : ViewSelector {
  void selectFeature(const FeatureList& views,
                     SemanticNodeAttributes& attrs) const override {
    double min_dist = std::numeric_limits<double>::max();
    const FeatureView* best_view = nullptr;
    for (const auto& view : views) {
      if (!view) {
        continue;
      }

      Eigen::Vector3d p_s;
      if (!view->pointInView(attrs.position, &p_s)) {
        continue;
      }

      // heuristic to pick the view that's closest to the boundary of the free-space
      // sphere
      auto derived = dynamic_cast<PlaceNodeAttributes*>(&attrs);
      double radius = derived ? derived->distance : 0.0;
      const auto dist = std::abs(radius - p_s.norm());
      if (dist < min_dist) {
        attrs.semantic_feature = view->feature;
        best_view = view.get();
        min_dist = dist;
      }
    }

    if (best_view) {
      attrs.semantic_feature = best_view->feature;
    }
  }

  inline static const auto registration_ =
      config::Registration<ViewSelector, BoundaryViewSelector>("boundary");
};

struct ClosestViewSelector : ViewSelector {
  void selectFeature(const FeatureList& views,
                     SemanticNodeAttributes& attrs) const override {
    const FeatureView* best_view = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& view : views) {
      if (!view) {
        continue;
      }

      Eigen::Vector3d p_s;
      if (!view->pointInView(attrs.position, &p_s)) {
        continue;
      }

      // norm of position in sensor frame is distance in world frame
      const auto dist = p_s.norm();
      if (dist < min_dist) {
        best_view = view.get();
        min_dist = dist;
      }
    }

    if (best_view) {
      attrs.semantic_feature = best_view->feature;
    }
  }

  inline static const auto registration_ =
      config::Registration<ViewSelector, ClosestViewSelector>("closest");
};

struct FusionViewSelector : ViewSelector {
  void selectFeature(const FeatureList& views,
                     SemanticNodeAttributes& attrs) const override {
    size_t num_visible = 0;
    for (const auto& view : views) {
      if (!view) {
        continue;
      }

      if (!view->pointInView(attrs.position)) {
        continue;
      }

      if (!num_visible) {
        attrs.semantic_feature = view->feature;
      } else {
        attrs.semantic_feature += view->feature;
      }
      ++num_visible;
    }

    if (num_visible > 0) {
      attrs.semantic_feature /= num_visible;
    }
  }

  inline static const auto registration_ =
      config::Registration<ViewSelector, FusionViewSelector>("fusion");
};

}  // namespace hydra
