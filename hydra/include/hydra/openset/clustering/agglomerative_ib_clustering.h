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
#include <config_utilities/virtual_config.h>
#include <spark_dsg/scene_graph_layer.h>

#include <Eigen/Dense>
#include <map>

#include "hydra/openset/layer_clustering.h"
#include "hydra/utils/logging.h"

namespace hydra {

class AgglomerativeIBClustering : public LayerClustering {
 public:
  struct ClusteringConfig : VerbosityConfig {
    float score_threshold = 0.23f;
    size_t top_k = 2;
    bool cumulative = true;
    bool null_task_preprune = true;
    double max_delta = 1.0e-3;
  };

  struct Config : LayerClustering::Config, ClusteringConfig {
    bool filter_clusters = false;
  } const config;

  struct Workspace {
    using Features = std::vector<FeatureVector>;
    using NodeEmbeddings = FeatureMap<spark_dsg::NodeId>;
    using ClusterIndices = std::vector<std::vector<spark_dsg::NodeId>>;

    Workspace(const ClusteringConfig& config,
              const spark_dsg::EdgeContainer::Edges& edges,
              const NodeEmbeddings& node_embeddings,
              const EmbeddingGroup& tasks,
              const EmbeddingDistance& metric);

    void reweight(double I_xy, double delta_weight);

    double score(const spark_dsg::EdgeKey& edge) const;

    bool merge(spark_dsg::EdgeKey to_merge,
               std::list<spark_dsg::EdgeKey>& updated,
               bool force = false);

    size_t size() const;

    size_t featureDim() const;

    std::string summary() const;

    ClusterIndices getClusters() const;

    static Eigen::MatrixXd compute_py_x(const ClusteringConfig& config,
                                        const Features& features,
                                        const EmbeddingGroup& tasks,
                                        const EmbeddingDistance& metric);

    const ClusteringConfig config;

    std::vector<FeatureVector> features;
    std::map<size_t, spark_dsg::NodeId> node_lookup;
    std::map<spark_dsg::NodeId, size_t> order;
    std::map<spark_dsg::EdgeKey, double> edges;
    std::vector<size_t> assignments;

    // p(x), p(z), p(y)
    Eigen::VectorXd px;
    Eigen::VectorXd pz;
    Eigen::VectorXd py;

    // p(z|x), p(y|x), p(y|z)
    Eigen::MatrixXd pz_x;  // NxN
    Eigen::MatrixXd py_x;  // 2xN
    Eigen::MatrixXd py_z;  // 2xN

    // mutual information caches
    double I_xy;
    double I_zy_prev;
    double delta_weight = 1.0;
    std::vector<double> deltas;
  };

  AgglomerativeIBClustering(const Config& config);

  Clusters cluster(const spark_dsg::SceneGraphLayer& layer) const override;

  static void cluster(Workspace& workspace, const VerbosityConfig& config = {});
};

void declare_config(AgglomerativeIBClustering::ClusteringConfig& config);
void declare_config(AgglomerativeIBClustering::Config& config);

}  // namespace hydra
