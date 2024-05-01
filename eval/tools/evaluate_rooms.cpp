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
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <filesystem>

#include "hydra/common/common.h"
#include "hydra/eval/room_evaluator.h"

DEFINE_string(tsdf_file, "", "tsdf file to read");
DEFINE_string(bbox_file, "", "bounding box config file");
DEFINE_string(dsg_file, "", "dsg file to read");
DEFINE_int32(min_room_nodes, 0, "minimum number of room nodes for evaluation");
DEFINE_bool(only_labeled, true, "only compute metrics for labeled voxels");

using namespace spark_dsg;

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility for comparing visualizing room bounding boxes");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  std::filesystem::path tsdf_path(FLAGS_tsdf_file);
  if (!std::filesystem::exists(tsdf_path)) {
    LOG(ERROR) << "TSDF file is required!";
    return 1;
  }

  std::filesystem::path bbox_path(FLAGS_bbox_file);
  if (!std::filesystem::exists(bbox_path)) {
    LOG(ERROR) << "Bounding box file is required!";
    return 1;
  }

  std::filesystem::path dsg_path(FLAGS_dsg_file);
  if (!std::filesystem::exists(dsg_path)) {
    LOG(ERROR) << "DSG file is required!";
    return 1;
  }

  hydra::eval::RoomEvaluator::Config config;
  config.only_labeled = FLAGS_only_labeled;
  config.min_room_nodes = FLAGS_min_room_nodes;
  auto evaluator = hydra::eval::RoomEvaluator::fromFile(config, bbox_path, tsdf_path);
  if (!evaluator) {
    LOG(ERROR) << "Unable to construct room evaluator.";
    return 1;
  }

  const auto metrics = evaluator->eval(dsg_path);
  if (!metrics.valid()) {
    LOG(ERROR) << "Unable to evaluate invalid graph!";
    return 1;
  }

  std::cout << metrics << std::endl;
  return 0;
}
