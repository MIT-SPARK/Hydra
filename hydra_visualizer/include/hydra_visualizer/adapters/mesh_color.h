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

#include <config_utilities/factory.h>
#include <spark_dsg/mesh.h>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra {

/**
 * @brief Functor to color a mesh.
 */
struct MeshColoring {
  using Ptr = std::shared_ptr<MeshColoring>;
  using ConstPtr = std::shared_ptr<const MeshColoring>;

  virtual ~MeshColoring() = default;

  /**
   * @brief Set the mesh that is about to be draw (to compute bounds)
   */
  virtual void setMesh(const spark_dsg::Mesh&) {}

  /**
   * @brief Function to color a mesh. Return the color of the i-th vertex. the base mesh
   * colorig will return the stored color of the mesh if it exists.
   */
  virtual spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh,
                                          size_t i) const = 0;
};

/**
 * @brief Functor to color a mesh with a single color.
 */
struct UniformMeshColoring : public MeshColoring {
  struct Config {
    spark_dsg::Color color;
  } const config;
  explicit UniformMeshColoring(const spark_dsg::Color& color);
  explicit UniformMeshColoring(const Config& config);
  virtual ~UniformMeshColoring() = default;

  spark_dsg::Color getVertexColor(const spark_dsg::Mesh&, size_t) const override {
    return config.color;
  }

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<MeshColoring, UniformMeshColoring, Config>(
          "UniformMeshColoring");
};

void declare_config(UniformMeshColoring::Config& config);

/**
 * @brief Functor to color a mesh based on the semantic label of the vertex.
 */
struct SemanticMeshColoring : public MeshColoring {
  struct Config {
    visualizer::CategoricalColormap::Config colormap;
  } const config;
  explicit SemanticMeshColoring(const Config& config);
  virtual ~SemanticMeshColoring() = default;

  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;

 private:
  const visualizer::CategoricalColormap colormap_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<MeshColoring, SemanticMeshColoring, Config>(
          "SemanticMeshColoring");
};

void declare_config(SemanticMeshColoring::Config& config);

/**
 * @brief Functor to color a mesh based on the first time a vertex was seen. The color
 * ranges from dark (early first seen) to light (late first seen), with unitialized
 * vertices being shown in green.
 *
 */
struct FirstSeenMeshColoring : public MeshColoring {
  struct Config {};
  FirstSeenMeshColoring();
  explicit FirstSeenMeshColoring(const Config&);
  virtual ~FirstSeenMeshColoring() = default;

  void setMesh(const spark_dsg::Mesh& mesh) override;
  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;
  void setBounds(spark_dsg::Mesh::Timestamp min, spark_dsg::Mesh::Timestamp max);

 private:
  spark_dsg::Mesh::Timestamp min_;
  spark_dsg::Mesh::Timestamp max_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<MeshColoring, FirstSeenMeshColoring, Config>(
          "FirstSeenMeshColoring");
};

void declare_config(FirstSeenMeshColoring::Config& config);

/**
 * @brief Functor to color a mesh based on the last time a vertex was seen. The color
 * ranges from dark (early last seen) to light (late last seen), with unitialized
 * vertices being shown in green.
 */
struct LastSeenMeshColoring : public MeshColoring {
  struct Config {};
  LastSeenMeshColoring();
  explicit LastSeenMeshColoring(const Config&);
  virtual ~LastSeenMeshColoring() = default;

  void setMesh(const spark_dsg::Mesh& mesh) override;
  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;
  void setBounds(spark_dsg::Mesh::Timestamp min, spark_dsg::Mesh::Timestamp max);

 private:
  spark_dsg::Mesh::Timestamp min_;
  spark_dsg::Mesh::Timestamp max_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<MeshColoring, LastSeenMeshColoring, Config>(
          "LastSeenMeshColoring");
};

void declare_config(LastSeenMeshColoring::Config& config);

/**
 * @brief Functor to color a mesh based on the duration a vertex was seen. The color
 * ranges from dark (short duration) to light (long duration), with invalid durations
 * being shown in green.
 */
struct SeenDurationMeshColoring : public MeshColoring {
  struct Config {};
  SeenDurationMeshColoring();
  explicit SeenDurationMeshColoring(const Config&);
  virtual ~SeenDurationMeshColoring() = default;

  void setMesh(const spark_dsg::Mesh& mesh) override;
  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;
  void setMaxDuration(spark_dsg::Mesh::Timestamp max);

 private:
  spark_dsg::Mesh::Timestamp max_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<MeshColoring, SeenDurationMeshColoring, Config>(
          "SeenDurationMeshColoring");
};

void declare_config(SeenDurationMeshColoring::Config& config);

/**
 * @brief Functor to partially apply a color functor
 *
 * Points with a positive distance to the plane will use the child coloring functor,
 * otherwise they will use the original mesh color
 */
struct SplitMeshColoring : public MeshColoring {
  struct Config {
    config::VirtualConfig<MeshColoring> coloring;
    Eigen::Vector3f normal = Eigen::Vector3f::Ones();
    Eigen::Vector3f origin = Eigen::Vector3f::Zero();
    spark_dsg::Color default_color = spark_dsg::Color::gray();
  } const config;

  explicit SplitMeshColoring(const Config& config);
  virtual ~SplitMeshColoring() = default;

  void setMesh(const spark_dsg::Mesh& mesh) override;
  spark_dsg::Color getVertexColor(const spark_dsg::Mesh& mesh, size_t i) const override;

 private:
  MeshColoring::Ptr coloring_;
};

void declare_config(SplitMeshColoring::Config& config);

/**
 * @brief Utility class to color a mesh based on a mesh coloring for visualization.
 */
struct MeshColorAdapter {
  /**
   * @brief Function to color a mesh. Return the color of the i-th vertex. If no
   * coloring is set, the color stored in the mesh is used.
   */
  explicit MeshColorAdapter(const spark_dsg::Mesh& mesh,
                            MeshColoring::ConstPtr coloring = nullptr);
  virtual ~MeshColorAdapter() = default;

  std::function<spark_dsg::Color(size_t)> getVertexColor;

  const spark_dsg::Mesh& mesh;

 private:
  const MeshColoring::ConstPtr coloring_;
};

spark_dsg::Color colorFromTime(spark_dsg::Mesh::Timestamp time,
                               spark_dsg::Mesh::Timestamp min,
                               spark_dsg::Mesh::Timestamp max);

}  // namespace hydra
