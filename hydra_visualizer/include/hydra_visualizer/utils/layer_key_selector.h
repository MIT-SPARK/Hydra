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

namespace hydra {

/*
 * @brief Type representing a specification for a layer and set of partitions
 *
 * Can be of the form
 *   - [LayerId] (e.g., '2'), which selects Layer 2, Partition 0
 *   - [LayerId]p[PartitionId] (e.g., '2p1'), which selects Layer 2, Partition 1
 *   - [LayerId]p* (e.g., '2p*'), which selects Layer 2, Partitions >= 1
 *   - [LayerId]* (e.g., '2*'), which selects Layer 2, Partitions >= 0
 */
struct LayerKeySelector {
  //! Layer and partition the selector references
  spark_dsg::LayerKey key;
  //! @brief whether or not the selector covers all partitions >= 1
  bool wildcard = false;
  //! @brief whether or not wildcard includes default partition (partition 0)
  bool include_default = false;

  static std::optional<LayerKeySelector> parse(const std::string& selector_str);

  std::string str() const;
  bool matches(spark_dsg::LayerKey to_match) const;
  bool operator<(const LayerKeySelector& other) const;
  bool operator==(const LayerKeySelector& other) const;
  bool operator!=(const LayerKeySelector& other) const;
};

struct SelectorConversion {
  static std::string toIntermediate(const LayerKeySelector& value, std::string& error);
  static void fromIntermediate(const std::string& intermediate,
                               LayerKeySelector& value,
                               std::string& error);
};

struct LayerKeyConversion {
  static std::string toIntermediate(const spark_dsg::LayerKey& value,
                                    std::string& error);
  static void fromIntermediate(const std::string& intermediate,
                               spark_dsg::LayerKey& value,
                               std::string& error);
};

}  // namespace hydra
