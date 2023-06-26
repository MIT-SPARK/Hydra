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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/rooms/room_finder_config.h>
#include <yaml-cpp/yaml.h>

namespace hydra {

using namespace config_parser;

TEST(DsgBuilderConfigTests, RoomFinderConfig) {
  const auto node = YAML::Load(R"yaml(
      room_prefix: r
      min_dilation_m: 0.2
      max_dilation_m: 0.8
      min_component_size: 10
      min_room_size: 5
      clustering_mode: NONE
      max_modularity_iters: 1
      modularity_gamma: 5.0
  )yaml");

  RoomFinderConfig config;
  YamlParser parser(std::make_unique<YamlParserImpl>(node));
  visit_config(parser, config);
  EXPECT_EQ(config.room_prefix, 'r');
  EXPECT_NEAR(config.min_dilation_m, 0.2, 1.0e-9);
  EXPECT_NEAR(config.max_dilation_m, 0.8, 1.0e-9);
  EXPECT_EQ(config.min_component_size, 10u);
  EXPECT_EQ(config.min_room_size, 5u);
  EXPECT_EQ(config.clustering_mode, RoomClusterMode::NONE);
  EXPECT_EQ(config.max_modularity_iters, 1u);
  EXPECT_NEAR(config.modularity_gamma, 5.0, 1.0e-9);

  VLOG(1) << config;
}

}  // namespace hydra
