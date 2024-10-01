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
#include "hydra/input/input_data.h"

namespace hydra {

struct SensorInputPacket {
  using Ptr = std::shared_ptr<SensorInputPacket>;

  explicit SensorInputPacket(uint64_t stamp, const std::string& sensor_name)
      : timestamp_ns(stamp), sensor_name(sensor_name) {}

  virtual ~SensorInputPacket() = default;

  bool fillInputData(InputData& msg) const;

  const uint64_t timestamp_ns;
  const std::string sensor_name;
  std::string sensor_frame;
  //! Learned feature for the input data (e.g., CLIP for camera)
  FeatureVector input_feature;

 protected:
  virtual bool fillInputDataImpl(InputData& msg) const = 0;
};

struct ImageInputPacket : public SensorInputPacket {
  explicit ImageInputPacket(uint64_t stamp, const std::string& sensor_name);

  //! Color for each pixel
  cv::Mat color;
  //! Depth for each pixel
  cv::Mat depth;
  //! Labels for each pixel
  cv::Mat labels;
  //! Features associated with each label
  FeatureMap<int> label_features;
  //! Whether or not the input color image is bgr order
  bool color_is_bgr = false;

 protected:
  bool fillInputDataImpl(InputData& msg) const override;
};

struct CloudInputPacket : public SensorInputPacket {
  explicit CloudInputPacket(uint64_t stamp, const std::string& sensor_name);

  bool in_world_frame = false;
  cv::Mat points;
  cv::Mat colors;
  cv::Mat labels;

 protected:
  bool fillInputDataImpl(InputData& msg) const override;
};

}  // namespace hydra
