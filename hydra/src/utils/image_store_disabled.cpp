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
// Image store of builds without HYDRA_ENABLE_IMAGE_STORE: no store can be opened, so
// every image writer is disabled
#include <glog/logging.h>

#include <mutex>

#include "hydra/utils/image_store.h"

namespace hydra {

struct ImageStore::Impl {};

bool ImageStore::enabled() { return false; }

ImageStore::Ptr ImageStore::open(const Config& config) {
  if (!config.path.empty()) {
    static std::once_flag warned;
    std::call_once(warned, [&config]() {
      LOG(WARNING) << "[ImageStore] Hydra was built without the image store "
                   << "(HYDRA_ENABLE_IMAGE_STORE=OFF), no images are saved to "
                   << config.path;
    });
  }

  return nullptr;
}

// open() never returns a store, so the members below are unreachable

ImageStore::ImageStore(const Config& config) : config(config) {}

ImageStore::~ImageStore() = default;

uint64_t ImageStore::addSensor(int, const std::string&, const CameraCalib&, double) {
  return 0;
}

uint64_t ImageStore::addFrame(const Frame&, const cv::Mat&, const cv::Mat&) {
  return 0;
}

std::vector<uint64_t> ImageStore::addObservations(int,
                                                  uint64_t,
                                                  const std::vector<Observation>&) {
  return {};
}

void ImageStore::setAnchor(uint64_t, spark_dsg::NodeId, const Eigen::Isometry3d&) {}

std::vector<ImageStore::PendingFrame> ImageStore::unanchoredFrames(int,
                                                                   uint64_t) const {
  return {};
}

void ImageStore::abandonAnchor(uint64_t) {}

void ImageStore::bindTrack(int, uint64_t, spark_dsg::NodeId) {}

void ImageStore::unbindTrack(int, uint64_t) {}

void ImageStore::recordMerges(const std::map<spark_dsg::NodeId, spark_dsg::NodeId>&,
                              const std::vector<spark_dsg::NodeId>&) {}

void ImageStore::flush() {}

}  // namespace hydra
