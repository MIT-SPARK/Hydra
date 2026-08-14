#include "hydra_ros/backend/gt_room_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra {

static const auto registration_ =
    config::RegistrationWithConfig<UpdateRoomsFunctor::Sink,
                                   GtRoomPublisher,
                                   GtRoomPublisher::Config>("GtRoomPublisher");

void declare_config(GtRoomPublisher::Config& config) {
  using namespace config;
  name("GtRoomPublisher::Config");
  field(config.ns, "ns");
  field(config.room_frame_id, "room_frame_id");
  field(config.colormap, "colormap");
}

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

GtRoomPublisher::GtRoomPublisher(const Config& config)
    : config(config), nh_(ianvs::NodeHandle::this_node(config.ns)) {
  room_publisher_ = nh_.create_publisher<MarkerArray>("gt_room_boundaries", 1);
}

std::string GtRoomPublisher::printInfo() const { return config::toString(config); }

void GtRoomPublisher::call(uint64_t, const RoomFinder& rf) const {
  MarkerArray ma;
  auto& m = ma.markers.emplace_back();
  m.action = m.DELETEALL;
  int idx = 0;
  int room_idx = 0;

  auto colormap = visualizer::DiscreteColormap(config.colormap);

  for (auto room : rf.room_extents.room_bounding_boxes) {
    for (auto box : room) {
      auto& m = ma.markers.emplace_back();
      m.header.frame_id = config.room_frame_id;
      m.ns = "gt_rooms";
      m.id = idx++;
      m.action = m.ADD;
      m.type = m.CUBE;

      Eigen::Quaternionf q(box.world_R_center);
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();
      m.pose.orientation.w = q.w();
      m.pose.position.x = box.world_P_center.x();
      m.pose.position.y = box.world_P_center.y();
      m.pose.position.z = box.world_P_center.z();
      m.scale.x = box.dimensions.x();
      m.scale.y = box.dimensions.y();
      m.scale.z = box.dimensions.z();
      m.color = visualizer::makeColorMsg(colormap.getColor(room_idx), 0.5);
    }
    ++room_idx;
  }

  room_publisher_->publish(ma);
}

}  // namespace hydra
