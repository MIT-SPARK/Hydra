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
#include "hydra/backend/backend_utilities.h"

#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <string>

namespace hydra::utils {

using spark_dsg::AgentNodeAttributes;
using spark_dsg::KhronosObjectAttributes;
using spark_dsg::NodeAttributes;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraph;

std::optional<uint64_t> getTimeNs(const SceneGraph& graph, gtsam::Symbol key) {
  NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node.str() << "when logging loop closure";
    return std::nullopt;
  }

  return graph.getNode(node).attributes<AgentNodeAttributes>().timestamp.count();
}

size_t moveImageFiles(const std::filesystem::path& src,
                      const std::filesystem::path& dest) {
  std::error_code ec;
  if (src == dest || !std::filesystem::exists(src, ec)) {
    return 0;
  }

  std::filesystem::create_directories(dest, ec);
  if (ec) {
    LOG(WARNING) << "Failed to create image folder " << dest << ": " << ec.message();
    return 0;
  }

  size_t moved = 0;
  for (const auto& entry : std::filesystem::directory_iterator(src, ec)) {
    const auto target = dest / entry.path().filename();
    std::error_code move_ec;
    if (std::filesystem::exists(target, move_ec)) {
      // rename would silently replace the existing file
      LOG(WARNING) << "Not moving " << entry.path() << ": " << target
                   << " already exists";
      continue;
    }

    std::filesystem::rename(entry.path(), target, move_ec);
    if (move_ec) {
      LOG(WARNING) << "Failed to move " << entry.path() << " to " << dest << ": "
                   << move_ec.message();
      continue;
    }

    ++moved;
  }

  // only removes src if empty, i.e., all files were moved
  std::filesystem::remove(src, ec);
  VLOG(2) << "Moved " << moved << " image file(s) from " << src << " to " << dest;
  return moved;
}

ObjectImageFolders::ObjectImageFolders(const std::filesystem::path& image_root)
    : image_root_(image_root),
      temp_root_(image_root.empty() ? image_root : image_root / kTempImageFolder) {}

std::filesystem::path ObjectImageFolders::finalPath(NodeId node) const {
  const NodeSymbol symbol(node);
  return image_root_ / (std::string(1, symbol.category()) + "_" +
                        std::to_string(symbol.categoryId()));
}

std::string ObjectImageFolders::finalFolder(NodeId node) const {
  return relativeImageFolder(image_root_, finalPath(node));
}

bool ObjectImageFolders::isTemporary(const std::string& folder) const {
  return enabled() && !folder.empty() &&
         isPathUnder(resolveImageFolder(image_root_, folder), temp_root_);
}

void ObjectImageFolders::update(const SceneGraph& unmerged,
                                const std::string& layer,
                                const std::map<NodeId, NodeId>& merges,
                                SceneGraph& merged) const {
  if (!enabled() || !unmerged.hasLayer(layer)) {
    return;
  }

  const auto& unmerged_layer = unmerged.getLayer(layer);
  const auto mirror = [&](NodeId node_id) {
    const auto target = merged.findNode(node_id);
    if (!target) {
      return;
    }

    const auto attrs = target->tryAttributes<KhronosObjectAttributes>();
    const auto folder = finalFolder(node_id);
    if (!attrs || attrs->image_folder == folder) {
      return;
    }

    auto new_attrs = attrs->clone();
    dynamic_cast<KhronosObjectAttributes&>(*new_attrs).image_folder = folder;
    merged.setNodeAttributes(node_id, std::move(new_attrs));
  };

  // union the folders of nodes merged since the last call into the surviving node
  for (const auto& [child, parent] : merges) {
    if (!unmerged_layer.hasNode(child)) {
      continue;
    }

    const auto prev = unioned_.find(child);
    if (prev != unioned_.end() && prev->second == parent) {
      continue;
    }

    unioned_[child] = parent;
    const auto dest = finalPath(parent);
    if (moveImageFiles(finalPath(child), dest) > 0) {
      mirror(parent);
    }
  }

  // the frontend's pointers to the temporary folders live on the unmerged graph (the
  // merged copy of an archived node is not refreshed); move the crops to the final
  // folder of the surviving node and mirror that folder onto the merged graph
  for (const auto& node : unmerged_layer.nodes()) {
    const auto attrs = node.tryAttributes<KhronosObjectAttributes>();
    if (!attrs || attrs->image_folder.empty()) {
      continue;
    }

    if (!isTemporary(attrs->image_folder)) {
      continue;
    }

    const auto iter = merges.find(node.id);
    const auto target_id = iter == merges.end() ? node.id : iter->second;
    // the unmerged graph keeps pointing to the temporary folder after the move, so
    // only touch the filesystem the first time a folder is seen for a node
    auto& moved_folder = moved_[node.id];
    if (moved_folder != attrs->image_folder) {
      moveImageFiles(resolveImageFolder(image_root_, attrs->image_folder),
                     finalPath(target_id));
      moved_folder = attrs->image_folder;
    }

    mirror(target_id);
  }
}

void ObjectImageFolders::finalize(NodeId node, NodeAttributes& attrs) const {
  auto derived = dynamic_cast<KhronosObjectAttributes*>(&attrs);
  if (!enabled() || !derived) {
    return;
  }

  const auto dest = finalPath(node);
  std::error_code ec;
  if (derived->image_folder.empty() && !std::filesystem::exists(dest, ec)) {
    return;  // node has no crops
  }

  derived->image_folder = finalFolder(node);
}

}  // namespace hydra::utils
