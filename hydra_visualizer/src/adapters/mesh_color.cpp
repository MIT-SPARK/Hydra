#include "hydra_visualizer/adapters/mesh_color.h"

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/validation.h>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {
namespace {

static const auto splt_reg =
    config::RegistrationWithConfig<MeshColoring,
                                   SplitMeshColoring,
                                   SplitMeshColoring::Config>("SplitMeshColoring");

}

using spark_dsg::Color;
using spark_dsg::Mesh;

void declare_config(UniformMeshColoring::Config& config) {
  using namespace config;
  name("UniformMeshColoring::Config");
  field(config.color, "color");
}

UniformMeshColoring::UniformMeshColoring(const spark_dsg::Color& color)
    : UniformMeshColoring(Config{color}) {}

UniformMeshColoring::UniformMeshColoring(const Config& config) : config(config) {}

void declare_config(SemanticMeshColoring::Config& config) {
  using namespace config;
  name("SemanticMeshColoring::Config");
  field(config.colormap, "colormap");
}

SemanticMeshColoring::SemanticMeshColoring(const Config& config)
    : config(config), colormap_(config.colormap) {}

Color SemanticMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  if (!mesh.has_labels) {
    return Color::black();
  }

  return colormap_.getColor(mesh.label(i));
}

void declare_config(FirstSeenMeshColoring::Config&) {
  config::name("FirstSeenMeshColoring::Config");
}

FirstSeenMeshColoring::FirstSeenMeshColoring() : FirstSeenMeshColoring(Config()) {}

FirstSeenMeshColoring::FirstSeenMeshColoring(const Config&) {}

void FirstSeenMeshColoring::setMesh(const Mesh& mesh) {
  min_ = std::numeric_limits<Mesh::Timestamp>::max();
  max_ = std::numeric_limits<Mesh::Timestamp>::min();
  for (const auto t : mesh.first_seen_stamps) {
    if (t < min_ && t > 0) {
      min_ = t;
    }
    if (t > max_) {
      max_ = t;
    }
  }
}

Color FirstSeenMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.first_seen_stamps[i], min_, max_);
}

void FirstSeenMeshColoring::setBounds(Mesh::Timestamp min, Mesh::Timestamp max) {
  min_ = min;
  max_ = max;
}

void declare_config(LastSeenMeshColoring::Config&) {
  config::name("LastSeenMeshColoring::Config");
}

LastSeenMeshColoring::LastSeenMeshColoring() : LastSeenMeshColoring(Config()) {}

LastSeenMeshColoring::LastSeenMeshColoring(const Config&) {}

void LastSeenMeshColoring::setMesh(const Mesh& mesh) {
  min_ = std::numeric_limits<Mesh::Timestamp>::max();
  max_ = std::numeric_limits<Mesh::Timestamp>::min();
  for (const auto t : mesh.stamps) {
    if (t < min_ && t > 0) {
      min_ = t;
    }
    if (t > max_) {
      max_ = t;
    }
  }
}

Color LastSeenMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.stamps[i], min_, max_);
}

void LastSeenMeshColoring::setBounds(Mesh::Timestamp min, Mesh::Timestamp max) {
  min_ = min;
  max_ = max;
}

void declare_config(SeenDurationMeshColoring::Config&) {
  config::name("SeenDurationMeshColoring::Config");
}

SeenDurationMeshColoring::SeenDurationMeshColoring()
    : SeenDurationMeshColoring(Config()) {}

SeenDurationMeshColoring::SeenDurationMeshColoring(const Config&) {}

void SeenDurationMeshColoring::setMesh(const Mesh& mesh) {
  max_ = 0;
  for (size_t i = 0; i < mesh.stamps.size(); ++i) {
    const auto duration = mesh.stamps[i] - mesh.first_seen_stamps[i];
    if (duration > max_) {
      max_ = duration;
    }
  }
}

Color SeenDurationMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.stamps[i] - mesh.first_seen_stamps[i], 0, max_);
}

void SeenDurationMeshColoring::setMaxDuration(spark_dsg::Mesh::Timestamp max) {
  max_ = max;
}

void declare_config(SplitMeshColoring::Config& config) {
  using namespace config;
  name("SplitMeshColoring::Config");
  config.coloring.setOptional();
  field(config.coloring, "coloring");
  field(config.normal, "normal");
  field(config.origin, "origin");
  field(config.default_color, "default_color");
}

SplitMeshColoring::SplitMeshColoring(const Config& config)
    : config(config::checkValid(config)), coloring_(config.coloring.create()) {}

void SplitMeshColoring::setMesh(const Mesh& mesh) {
  if (coloring_) {
    coloring_->setMesh(mesh);
  }
}

Color SplitMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  const auto& pos = mesh.pos(i);
  const auto dist = config.normal.dot(pos - config.origin);
  if (dist >= 0.0f && coloring_) {
    return coloring_->getVertexColor(mesh, i);
  }

  return mesh.has_colors ? mesh.color(i) : config.default_color;
}

MeshColorAdapter::MeshColorAdapter(const Mesh& mesh, MeshColoring::ConstPtr coloring)
    : mesh(mesh), coloring_(std::move(coloring)) {
  if (coloring_) {
    const_cast<MeshColoring&>(*coloring_).setMesh(mesh);
    getVertexColor = [this](size_t i) {
      return coloring_->getVertexColor(this->mesh, i);
    };
  } else {
    getVertexColor = [this](size_t i) { return this->mesh.color(i); };
  }
}

Color colorFromTime(Mesh::Timestamp time, Mesh::Timestamp min, Mesh::Timestamp max) {
  static const visualizer::IronbowPalette palette;
  if (time == 0) {
    return Color::green();
  }

  if (time <= min) {
    return palette(0);
  } else if (time >= max) {
    return palette(1);
  }

  return palette(static_cast<double>(time - min) / static_cast<double>(max - min));
}

}  // namespace hydra
