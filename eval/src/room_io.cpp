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
#include "hydra/eval/room_io.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace YAML {

namespace {
inline std::vector<float> fromEigen(const Eigen::Vector3f& vec) {
  return {vec.x(), vec.y(), vec.z()};
}

inline bool toEigen(const std::vector<float>& lhs, Eigen::Vector3f& rhs) {
  if (lhs.size() != 3) {
    return false;
  }
  rhs << lhs.at(0), lhs.at(1), lhs.at(2);
  return true;
}

inline std::map<std::string, float> fromEigen(const Eigen::Quaternionf& q) {
  return {{"w", q.w()}, {"x", q.x()}, {"y", q.y()}, {"z", q.z()}};
}

inline bool toEigen(const std::map<std::string, float>& lhs, Eigen::Quaternionf& rhs) {
  if (!lhs.count("w") || !lhs.count("x") || !lhs.count("y") || !lhs.count("z")) {
    return false;
  }
  rhs = Eigen::Quaternionf(lhs.at("w"), lhs.at("x"), lhs.at("y"), lhs.at("z"));
  return true;
}

}  // namespace

template <>
struct convert<spark_dsg::BoundingBox> {
  static Node encode(const spark_dsg::BoundingBox& rhs) {
    Node node;
    node["center"] = fromEigen(rhs.world_P_center);
    node["extents"] = fromEigen(rhs.dimensions);
    node["rotation"] = fromEigen(Eigen::Quaternionf(rhs.world_R_center));
    return node;
  }

  static bool decode(const Node& node, spark_dsg::BoundingBox& rhs) {
    if (!node.IsMap()) {
      return false;
    }

    if (!node["center"] || !node["extents"] || !node["rotation"]) {
      return false;
    }

    Eigen::Vector3f pos;
    if (!toEigen(node["center"].as<std::vector<float>>(), pos)) {
      return false;
    }

    Eigen::Vector3f scale;
    if (!toEigen(node["extents"].as<std::vector<float>>(), scale)) {
      return false;
    }

    Eigen::Quaternionf rot;
    if (!toEigen(node["rotation"].as<std::map<std::string, float>>(), rot)) {
      return false;
    }

    rhs = spark_dsg::BoundingBox(
        spark_dsg::BoundingBox::Type::OBB, scale, pos, rot.toRotationMatrix());
    return true;
  }
};

}  // namespace YAML

namespace hydra::eval {

RoomGeometry parseRoomGeometry(const YAML::Node& root) {
  if (!root.IsMap()) {
    throw std::runtime_error("invalid bounding box configuration!");
  }

  RoomGeometry geometries;
  const auto box_map = root.as<std::map<size_t, std::list<BoundingBox>>>();
  for (auto&& [room_id, boxes] : box_map) {
    if (!geometries.addRoom(room_id, boxes)) {
      LOG(WARNING) << "Founding repeated room id: " << room_id;
    }
  }

  return geometries;
}

RoomGeometry::RoomGeometry() {}

bool RoomGeometry::addRoom(size_t room_id, const std::list<BoundingBox>& boxes) {
  return rooms_.emplace(room_id, boxes).second;
}

std::optional<size_t> RoomGeometry::findRoomIndex(const Eigen::Vector3f& pos) const {
  for (auto&& [room_id, boxes] : rooms_) {
    for (const auto& box : boxes) {
      if (box.contains(pos)) {
        return room_id;
      }
    }
  }

  return std::nullopt;
}

std::vector<size_t> RoomGeometry::getRoomIds() const {
  std::vector<size_t> room_ids;
  for (const auto& id_room_pair : rooms_) {
    room_ids.push_back(id_room_pair.first);
  }
  return room_ids;
}

RoomGeometry RoomGeometry::fromFile(const std::string& filename) {
  const auto node = YAML::LoadFile(filename);
  return parseRoomGeometry(node);
}

RoomGeometry RoomGeometry::fromYaml(const std::string& contents) {
  const auto node = YAML::Load(contents);
  return parseRoomGeometry(node);
}

}  // namespace hydra::eval
