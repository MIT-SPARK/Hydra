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

#include <gtsam/inference/Symbol.h>
#include <kimera_pgmo/mesh_offset_info.h>
#include <spark_dsg/scene_graph.h>

#include <filesystem>
#include <map>
#include <string>

#include "hydra/utils/image_folder.h"

namespace hydra::utils {

std::optional<uint64_t> getTimeNs(const spark_dsg::SceneGraph& graph,
                                  gtsam::Symbol key);

/**
 * @brief Move every file in src into dest (creating dest if needed) and remove src.
 *
 * No-op if src and dest are the same or src does not exist. Files whose name already
 * exists in dest are not moved (src is then kept) and a warning is logged.
 * @returns Number of files moved
 */
size_t moveImageFiles(const std::filesystem::path& src,
                      const std::filesystem::path& dest);

/**
 * @brief Bookkeeping for per-object image folders written by the frontend.
 *
 * The frontend writes image crops for each object track to a temporary folder under
 * `<image_root>/temp` (see kTempImageFolder) and points the node's image_folder at it.
 * This moves the crops to a stable per-node folder `<image_root>/<prefix>_<index>`,
 * unions the folders of merged nodes into the surviving node's folder and mirrors the
 * final folder onto the backend graph. Attributes other than image_folder are never
 * touched. An empty image_root disables all of this. Stored image folders are relative
 * to the parent of image_root (see imageFolderBase), e.g., `images/temp/<track>` and
 * `images/O_<index>`; absolute values are also accepted.
 */
class ObjectImageFolders {
 public:
  explicit ObjectImageFolders(const std::filesystem::path& image_root);

  //! @brief Whether an image root was configured
  bool enabled() const { return !image_root_.empty(); }

  //! @brief Final image folder for a node on disk
  std::filesystem::path finalPath(spark_dsg::NodeId node) const;

  //! @brief Final image folder value stored for a node (relative)
  std::string finalFolder(spark_dsg::NodeId node) const;

  //! @brief Whether a stored image folder value is a temporary frontend folder
  bool isTemporary(const std::string& folder) const;

  /**
   * @brief Move temporary folders and union merged folders on disk and update the
   * image folders of the merged graph.
   * @param unmerged Unmerged graph (whose image folders point to the frontend output)
   * @param layer Layer to update
   * @param merges Mapping from merged node to surviving node
   * @param merged Merged graph to update
   */
  void update(const spark_dsg::SceneGraph& unmerged,
              const std::string& layer,
              const std::map<spark_dsg::NodeId, spark_dsg::NodeId>& merges,
              spark_dsg::SceneGraph& merged) const;

  //! @brief Point non-empty image folders of node attributes to the node's final path
  void finalize(spark_dsg::NodeId node, spark_dsg::NodeAttributes& attrs) const;

 private:
  const std::filesystem::path image_root_;
  const std::filesystem::path temp_root_;
  //! Surviving node each merged node's folder was last moved to (merges can be
  //! recomputed, so a node may later be merged into a different node)
  mutable std::map<spark_dsg::NodeId, spark_dsg::NodeId> unioned_;
  //! Temporary folder already moved per node (the frontend writes each folder once)
  mutable std::map<spark_dsg::NodeId, std::string> moved_;
};

/**
 * @brief Fill empty image folders of agent nodes from the keyframe images on disk.
 *
 * Agent keyframe images are saved as `<agent_dir>/agent_<timestamp_ns>*`, possibly
 * after the corresponding node was archived, so the folder can be missing from a graph
 * that skips archived attribute updates (e.g., when the frontend stopped before the
 * update reached the backend). The prefix is reconstructed from the node timestamp and
 * only set if `<prefix>_meta.json` exists. The stored prefix is relative to the parent
 * of agent_dir, e.g., `agents/agent_<timestamp_ns>`.
 * @param graph Graph to update
 * @param agent_dir Directory containing the keyframe images (empty is a no-op)
 * @returns Number of image folders filled
 */
size_t reconcileAgentImageFolders(spark_dsg::SceneGraph& graph,
                                  const std::filesystem::path& agent_dir);

/**
 * @brief Fill empty image folders of object nodes with the final per-node folder (see
 * ObjectImageFolders) if that folder exists on disk.
 * @param graph Graph to update
 * @param image_root Root of the object image folders (empty is a no-op)
 * @returns Number of image folders filled
 */
size_t reconcileObjectImageFolders(spark_dsg::SceneGraph& graph,
                                   const std::filesystem::path& image_root);

template <typename T>
void mergeIndices(const T& from, T& to) {
  std::vector<typename T::value_type> from_indices(from.begin(), from.end());
  std::vector<typename T::value_type> to_indices(to.begin(), to.end());
  to.clear();

  std::sort(from_indices.begin(), from_indices.end());
  std::sort(to_indices.begin(), to_indices.end());
  std::set_union(from_indices.begin(),
                 from_indices.end(),
                 to_indices.begin(),
                 to_indices.end(),
                 std::back_inserter(to));
}

}  // namespace hydra::utils
