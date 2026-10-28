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
#pragma once

#include <spark_dsg/scene_graph_types.h>

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <string>
#include <vector>

#include "hydra/utils/image_store_config.h"
#include "hydra/utils/keyframe_images.h"

namespace hydra {

/**
 * @brief Id-keyed store for the images associated with scene graph nodes.
 *
 * The store is a directory holding an SQLite index (store.sqlite, see
 * resources/image_store/schema.sql) and the image files it references (paths relative
 * to the directory):
 * - frames/r<robot>/<timestamp_ns>_rgb.jpg and _depth.png (16-bit millimeters)
 * - masks/r<robot>/<obs_id>.png (masks of kept object observations)
 *
 * Frame, observation and sensor ids are `(robot_id << 48) | sequence` and resume from
 * the largest id in the database when a store is reopened. Other processes (e.g., the
 * python package) may write to the same store: each batch is committed in an immediate
 * transaction that first advances the sequences past the ids in the database, and an id
 * that another writer took in the meantime is replaced when it is committed (a frame
 * another writer added for the same sensor and timestamp is reused instead).
 *
 * All writes are asynchronous: calls validate and assign ids immediately and queue the
 * work for a single writer thread, which encodes the images and commits the rows in
 * batched transactions. Image files are always written before the rows referencing them
 * are committed, so readers (which may open the database concurrently, it uses WAL)
 * never see a path to a missing file. Pending work is drained when the store is
 * destroyed. Producers block when the queue is full.
 *
 * There is one instance per directory and process (see open()), so all modules of a
 * process share its connection and writer thread. Pipeline modules get the store
 * configured by PipelineConfig::image_store through GlobalInfo::imageStore().
 */
class ImageStore {
 public:
  using Ptr = std::shared_ptr<ImageStore>;

  //! Version of the schema written by this implementation
  static constexpr int kSchemaVersion = 1;
  //! Track ids written by the pipeline are below this value; larger ids are reserved
  //! for external writers
  static constexpr uint64_t kMaxTrackId = uint64_t(1) << 40;
  //! Name of the database file in the store directory
  static constexpr char kDatabaseName[] = "store.sqlite";

  using Config = ImageStoreConfig;
  using OnExisting = ImageStoreConfig::OnExisting;

  //! Kinds of frames (stored as integers)
  enum class FrameKind : int { Keyframe = 0, SubKeyframe = 1, Observation = 2 };

  //! A camera frame (images are passed separately)
  struct Frame {
    int robot_id = 0;
    //! Sensor id returned by addSensor
    uint64_t sensor_id = 0;
    uint64_t timestamp_ns = 0;
    FrameKind kind = FrameKind::Keyframe;
    //! Odometric body pose of the frame
    Eigen::Isometry3d world_T_body = Eigen::Isometry3d::Identity();
    //! Agent node the frame is anchored to (if known)
    std::optional<spark_dsg::NodeId> anchor_node;
    //! Body pose relative to the anchor node (only used with anchor_node)
    Eigen::Isometry3d anchor_T_body = Eigen::Isometry3d::Identity();
  };

  //! One observation of a tracked object in a frame
  struct Observation {
    //! Frame returned by addFrame
    uint64_t frame_id = 0;
    uint64_t timestamp_ns = 0;
    //! Inclusive pixel bounds [min_x, min_y, max_x, max_y]
    std::optional<std::array<int, 4>> bbox2d;
    //! Axis-aligned 3D bounds (min, max) in the odometric world frame
    std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> bbox3d;
    //! Run-length encoded mask (see encodeMaskRle)
    std::string mask_rle;
    //! Selection score (larger is better)
    double score = 0.0;
    //! Whether the observation keeps its images (the frame's images and mask)
    bool kept = false;
    //! Mask written for kept observations (8-bit, nonzero is the object)
    cv::Mat mask;
  };

  //! A frame waiting for an anchor (see unanchoredFrames)
  struct PendingFrame {
    uint64_t frame_id = 0;
    uint64_t timestamp_ns = 0;
    Eigen::Isometry3d world_T_body = Eigen::Isometry3d::Identity();
  };

  /**
   * @brief Get the store for a directory, opening it if needed.
   *
   * Returns nullptr if the path is empty (the store is disabled). Returns the instance
   * already open for the same directory if there is one (its configuration is kept).
   * Returns nullptr (and logs the error) if the store cannot be opened, e.g., if the
   * database has a different schema version.
   *
   * The first open of a directory in a process applies Config::on_existing to a
   * non-empty directory (exactly once: later opens, also after every instance was
   * released, find the store of this process's session).
   */
  static Ptr open(const Config& config);

  //! @brief Open a directory with the default configuration
  static Ptr open(const std::filesystem::path& path);

  //! @brief Make an id from a robot id and a per-robot sequence number
  static uint64_t makeId(int robot_id, uint64_t sequence);

  //! @brief Run-length encode a mask: JSON array of run lengths over the row-major
  //! image alternating background and foreground (nonzero), starting with background
  static std::string encodeMaskRle(const cv::Mat& mask);

  ~ImageStore();

  const Config config;

  //! @brief Directory of the store
  const std::filesystem::path& path() const { return config.path; }

  //! @brief Register a camera (idempotent per robot and name; updates the calibration)
  uint64_t addSensor(int robot_id,
                     const std::string& name,
                     const CameraCalib& calib,
                     double depth_scale = kKeyframeDepthScale);

  /**
   * @brief Add a frame and its images.
   *
   * Frames are unique per sensor and timestamp. Adding an existing frame fills in
   * images it does not have yet, sets its anchor if it has none and keeps the lowest
   * kind. Frames without anchor are listed by unanchoredFrames().
   * @param frame Frame to add
   * @param color Color image (BGR or mono 8-bit, see colorToKeyframe; may be empty)
   * @param depth Depth image (16-bit millimeters or metric float; may be empty)
   * @returns Frame id
   */
  uint64_t addFrame(const Frame& frame,
                    const cv::Mat& color = cv::Mat(),
                    const cv::Mat& depth = cv::Mat());

  /**
   * @brief Record observations of a tracked object (masks of kept observations are
   * written as images).
   * @returns Observation ids (in input order)
   */
  std::vector<uint64_t> addObservations(int robot_id,
                                        uint64_t track_id,
                                        const std::vector<Observation>& observations);

  //! @brief Anchor a frame to an agent node
  void setAnchor(uint64_t frame_id,
                 spark_dsg::NodeId anchor_node,
                 const Eigen::Isometry3d& anchor_T_body);

  //! @brief Frames of a robot without anchor up to a timestamp (in time order)
  std::vector<PendingFrame> unanchoredFrames(int robot_id,
                                             uint64_t max_timestamp_ns) const;

  //! @brief Stop listing a frame as unanchored (readers use its odometric pose)
  void abandonAnchor(uint64_t frame_id);

  //! @brief Record which node a track belongs to (idempotent upsert)
  void bindTrack(int robot_id, uint64_t track_id, spark_dsg::NodeId node);

  //! @brief Forget a track's node (the node was deleted before the track was extracted)
  void unbindTrack(int robot_id, uint64_t track_id);

  /**
   * @brief Record node merges (child -> final parent, see SharedDsgInfo::merges).
   * @param merges Merges to add or update
   * @param removed Children that are no longer merged
   */
  void recordMerges(const std::map<spark_dsg::NodeId, spark_dsg::NodeId>& merges,
                    const std::vector<spark_dsg::NodeId>& removed = {});

  //! @brief Block until all queued work is committed
  void flush();

 private:
  explicit ImageStore(const Config& config);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hydra
