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

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace hydra {

/**
 * @brief Configuration of the image store (see ImageStore).
 *
 * Kept separate from image_store.h so that configs can hold it without depending on
 * OpenCV or SQLite.
 */
struct ImageStoreConfig {
  //! What the first open of a directory in a process does with a store that already
  //! holds files (see on_existing)
  enum class OnExisting { Append, Rotate, Fail };

  //! Directory of the store (created if missing). Empty disables the store
  std::filesystem::path path;
  /**
   * Handling of a non-empty store directory at its first open in this process.
   * Scene graph node ids and track ids restart with every Hydra session, so the
   * anchors, track bindings and node aliases of an earlier session can collide with
   * the new ones.
   * - Append (default): keep writing into the store (logs a warning).
   * - Rotate: move the directory to <parent>/sessions/<k>/<store name> (k = 1, 2, ...
   *   is one more than the largest existing session), copy session_artifacts next to
   *   it and write <parent>/sessions/<k>/session.json. The new session starts with an
   *   empty store at the configured path.
   * - Fail: refuse to open the store.
   */
  OnExisting on_existing = OnExisting::Append;
  //! Output directory of the earlier session (Hydra's log_path). On rotation, the
  //! files session_artifacts (relative to it) are copied to
  //! <parent>/sessions/<k>/<directory name>/ so the rotated store's node ids remain
  //! resolvable. Empty: nothing is copied.
  std::filesystem::path session_artifacts_dir;
  //! Files copied from session_artifacts_dir on rotation (missing ones are skipped)
  std::vector<std::string> session_artifacts = {"backend/dsg.json",
                                                "backend/dsg_with_mesh.json",
                                                "backend/mesh.ply",
                                                "backend/deformation_graph.dgrf",
                                                "backend/trajectory.csv",
                                                "backend/loop_closures.csv",
                                                "frontend/dsg.json",
                                                "frontend/dsg_with_mesh.json",
                                                "frontend/mesh.ply"};
  //! Number of observations per object track that keep their images
  size_t max_images_per_object = 1;
  //! Quality [0, 100] of the JPEG color images
  int jpeg_quality = 90;
  //! Whether depth images are written
  bool write_depth = true;
  //! Maximum time [ms] queued work waits before it is committed
  size_t flush_period_ms = 500;
  //! Maximum number of queued operations before producers block
  size_t max_queue_size = 256;
};

void declare_config(ImageStoreConfig& config);

}  // namespace hydra
