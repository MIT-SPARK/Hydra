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
#include "hydra/eval/room_metrics.h"

#include <glog/logging.h>

#include <numeric>

namespace hydra::eval {

bool RoomMetrics::valid() const { return !gt_sizes.empty() || !est_sizes.empty(); }

Eigen::MatrixXd computeOverlap(const RoomIndices& gt_rooms,
                               const RoomIndices& est_rooms) {
  Eigen::MatrixXd overlaps = Eigen::MatrixXd::Zero(gt_rooms.size(), est_rooms.size());

  size_t gt_idx = 0;
  for (const auto& gt_room_pair : gt_rooms) {
    size_t est_idx = 0;
    for (const auto& est_room_pair : est_rooms) {
      for (const auto& index : est_room_pair.second) {
        if (gt_room_pair.second.count(index)) {
          overlaps(gt_idx, est_idx) += 1.0;
        }
      }

      ++est_idx;
    }

    ++gt_idx;
  }

  return overlaps;
}

std::vector<size_t> getSizes(const RoomIndices& rooms) {
  std::vector<size_t> sizes;
  for (const auto& id_room_pair : rooms) {
    sizes.push_back(id_room_pair.second.size());
  }
  return sizes;
}

RoomMetrics scoreRooms(const RoomIndices& gt_rooms, const RoomIndices& est_rooms) {
  RoomMetrics results;
  results.gt_sizes = getSizes(gt_rooms);
  results.est_sizes = getSizes(est_rooms);
  results.overlaps = computeOverlap(gt_rooms, est_rooms);

  // recalls
  const auto max_gt_overlaps = results.overlaps.rowwise().maxCoeff();
  CHECK_EQ(max_gt_overlaps.rows(), static_cast<int>(results.gt_sizes.size()));

  for (size_t i = 0; i < results.gt_sizes.size(); ++i) {
    const auto curr_size = results.gt_sizes.at(i);
    results.recalls.push_back(curr_size ? max_gt_overlaps(i) / curr_size : 0.0);
  }

  const auto total_gt_size =
      std::accumulate(results.gt_sizes.begin(), results.gt_sizes.end(), 0);
  results.total_recall = total_gt_size ? max_gt_overlaps.sum() / total_gt_size : 0.0;

  // precisions
  const auto max_est_overlaps = results.overlaps.colwise().maxCoeff();
  CHECK_EQ(max_est_overlaps.cols(), static_cast<int>(results.est_sizes.size()));

  for (size_t i = 0; i < results.est_sizes.size(); ++i) {
    const auto curr_size = results.est_sizes.at(i);
    results.precisions.push_back(curr_size ? max_est_overlaps(i) / curr_size : 0.0);
  }

  Eigen::IOFormat fmt(6, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
  VLOG(10) << "GT: " << max_gt_overlaps.format(fmt);
  VLOG(10) << "EST: " << max_est_overlaps.format(fmt);

  const auto total_est_size =
      std::accumulate(results.est_sizes.begin(), results.est_sizes.end(), 0);
  results.total_precision =
      total_est_size ? max_est_overlaps.sum() / total_est_size : 0.0;
  return results;
}

template <typename T>
std::string showVector(const std::vector<T>& vec) {
  std::stringstream ss;
  ss << "[";
  auto iter = vec.begin();
  while (iter != vec.end()) {
    ss << *iter;
    ++iter;
    if (iter != vec.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

std::string showMatrix(const Eigen::MatrixXd& mat) {
  std::stringstream ss;
  ss << "[";
  for (Eigen::Index r = 0; r < mat.rows(); ++r) {
    ss << "[";
    for (Eigen::Index c = 0; c < mat.cols(); ++c) {
      ss << mat(r, c);
      if (c < mat.cols() - 1) {
        ss << ", ";
      }
    }
    ss << "]";
    if (r < mat.rows() - 1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const RoomMetrics& results) {
  out << "{";
  out << "\n  \"total_precision\": " << results.total_precision;
  out << ",\n  \"total_recall\": " << results.total_recall;
  out << ",\n  \"precisions\": " << showVector(results.precisions);
  out << ",\n  \"recalls\": " << showVector(results.recalls);
  out << ",\n  \"gt_sizes\": " << showVector(results.gt_sizes);
  out << ",\n  \"est_sizes\": " << showVector(results.est_sizes);
  out << ",\n  \"overlaps\": " << showMatrix(results.overlaps);
  out << "\n}";
  return out;
}

}  // namespace hydra::eval
