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

#include <config_utilities/parsing/context.h>
#include <hydra/openset/embedding_distances.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <memory>

namespace hydra {

/**
 * @brief Utility struct to globally manage cognition label lookup via a singleton.
 */
struct CognitionLabels {
  struct Config {
    //! Namespace of the features in the dsg meta_data.
    std::string ns = "features";

    //! Which feature type to read.
    enum class FeatureType {
      CLIP = 0,
      SENTENCE = 1,
      BOTH = 2
    } feature_type = FeatureType::CLIP;

    //! Distance metric to use for comparing features.
    config::VirtualConfig<hydra::EmbeddingDistance> distance_metric{
        hydra::CosineDistance::Config()};
  } const config;

  static CognitionLabels& instance();

  static void setup(
      const Config& config = config::fromContext<Config>("cognition_labels"));

  // Lookup.
  static FeatureVector getFeature(const spark_dsg::DynamicSceneGraph& dsg, int id);
  static float getScore(const spark_dsg::DynamicSceneGraph& dsg, int id1, int id2);
  static float getScore(const FeatureVector& f1, const FeatureVector& f2);

 private:
  explicit CognitionLabels(const Config& config);
  std::unique_ptr<hydra::EmbeddingDistance> distance_metric_;
  static std::unique_ptr<CognitionLabels> instance_;
};

struct LazyCognitionLabels {
  explicit LazyCognitionLabels(const spark_dsg::DynamicSceneGraph& dsg) : dsg_(dsg) {}

  const FeatureVector& get(int id) const;

 private:
  const spark_dsg::DynamicSceneGraph& dsg_;
  mutable std::unordered_map<int, FeatureVector> cache_;
};

// Returns the label with the highest score and its confidence from observations <label,
// weight>
std::pair<int, float> getMaxCognitionLabel(const std::map<int, float>& labels);

void declare_config(CognitionLabels::Config& config);

}  // namespace hydra