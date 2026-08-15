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
#include "hydra_visualizer/utils/marker_tracker.h"

namespace hydra {

using std_msgs::msg::Header;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

inline Marker makeDeleteMarker(const Header& header, size_t id, const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.action = Marker::DELETE;
  marker.id = id;
  marker.ns = ns;
  return marker;
}

inline bool isValid(const Marker& marker) {
  if (marker.type != Marker::LINE_STRIP && marker.type != Marker::LINE_LIST &&
      marker.type != Marker::CUBE_LIST && marker.type != Marker::SPHERE_LIST &&
      marker.type != Marker::POINTS && marker.type != Marker::TRIANGLE_LIST) {
    return true;
  }

  return !marker.points.empty();
}

}  // namespace

void MarkerTracker::add(const Marker& marker, MarkerArray& msg) {
  if (!isValid(marker)) {
    remove(marker.header, marker.ns, marker.id, msg);
    return;
  }

  msg.markers.push_back(marker);

  auto iter = published_markers_.find(marker.ns);
  if (iter == published_markers_.end()) {
    iter = published_markers_.emplace(marker.ns, std::set<size_t>()).first;
  }

  iter->second.insert(marker.id);
}

void MarkerTracker::add(const MarkerArray& markers, MarkerArray& msg) {
  for (const auto& marker : markers.markers) {
    add(marker, msg);
  }
}

void MarkerTracker::remove(const Header& header,
                           const std::string& ns,
                           size_t marker_id,
                           MarkerArray& msg) {
  auto iter = published_markers_.find(ns);
  if (iter == published_markers_.end()) {
    return;
  }

  Marker delete_marker = makeDeleteMarker(header, marker_id, ns);
  msg.markers.push_back(delete_marker);

  iter->second.erase(marker_id);
  if (iter->second.empty()) {
    published_markers_.erase(iter);
  }
}

void MarkerTracker::clearPrevious(const Header& header, MarkerArray& msg) {
  std::map<std::string, std::set<size_t>> previous = published_markers_;
  for (const auto& marker : msg.markers) {
    auto iter = previous.find(marker.ns);
    if (iter == previous.end()) {
      continue;
    }

    iter->second.erase(marker.id);
    if (iter->second.empty()) {
      previous.erase(iter);
    }
  }

  for (auto& [ns, prev_ids] : previous) {
    for (const auto id : prev_ids) {
      remove(header, ns, id, msg);
    }
  }
}

}  // namespace hydra
