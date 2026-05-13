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

#include <memory>

#include "hydra/active_window/active_window_output.h"
#include "hydra/active_window/volumetric_window.h"
#include "hydra/common/output_sink.h"
#include "hydra/places/gvd_places/graph_extractor.h"
#include "hydra/places/gvd_places/gvd_integrator.h"
#include "hydra/reconstruction/tsdf_interpolators.h"

namespace hydra {

class GvdPlaceExtractor {
 public:
  using Sink = OutputSink<uint64_t,
                          const Eigen::Isometry3d&,
                          const places::GvdLayer&,
                          const places::GraphExtractor&>;

  struct Config : VerbosityConfig {
    //! Node prefix to use
    char node_prefix = 'p';
    //! Target layer to add places
    std::string layer = spark_dsg::DsgLayers::PLACES;
    //! GVD integrator from TSDF
    places::GvdIntegrator::Config gvd;
    //! Graph extractor for processing GVD
    places::GraphExtractor::Config graph;
    //! Optional TSDF interpolator for downsampling TSDF
    config::VirtualConfig<TsdfInterpolator> tsdf_interpolator;
    //! Minimum number of places to keep a connected component
    size_t min_component_size = 3;
    //! Sinks for current pose and graph status
    std::vector<Sink::Factory> sinks;

    Config() : VerbosityConfig("[gvd_places] ") {}
  } const config;

  explicit GvdPlaceExtractor(const Config& config);

  virtual ~GvdPlaceExtractor();

  void detect(const ActiveWindowOutput& msg);

  void updateGraph(uint64_t timestamp_ns, spark_dsg::SceneGraph& graph);

 protected:
  void filterIsolated(spark_dsg::SceneGraph& graph,
                      std::set<spark_dsg::NodeId>& active_neighborhood);

 protected:
  places::GvdLayer::Ptr gvd_;
  std::unique_ptr<VolumetricWindow> map_window_;
  std::unique_ptr<TsdfInterpolator> tsdf_interpolator_;
  std::unique_ptr<places::GraphExtractor> graph_extractor_;
  std::unique_ptr<places::GvdIntegrator> gvd_integrator_;
  Sink::List sinks_;
};

void declare_config(GvdPlaceExtractor::Config& config);

}  // namespace hydra
