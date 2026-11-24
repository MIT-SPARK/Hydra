#include <gtest/gtest.h>

#include "hydra/common/hydra_pipeline.h"
#include "hydra/common/run_settings.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"

namespace hydra {

TEST(RunSettings, InheritedThreadsPersistAndExplicitOverridesTakePrecedence) {
  const auto previous = RunSettings::instance();
  ProjectiveIntegrator::Config inherited;
  MeshIntegrator::Config mesh_config;
  RunContext::Config first;
  first.settings.default_num_threads = 2;
  first.robot.robot_id = 1;
  const auto context = std::make_shared<RunContext>(first);
  EXPECT_EQ(MeshIntegrator(mesh_config).config.integrator_threads, 2);
  EXPECT_EQ(ProjectiveIntegrator(inherited).config.num_threads, 2);

  RunContext::Config second;
  second.robot.robot_id = 3;
  const RunContext next(second);
  EXPECT_EQ(ProjectiveIntegrator(inherited).config.num_threads, 2);

  auto explicit_config = inherited;
  explicit_config.num_threads = 1;
  EXPECT_EQ(ProjectiveIntegrator(explicit_config).config.num_threads, 1);

  mesh_config.integrator_threads = 1;
  EXPECT_EQ(MeshIntegrator(mesh_config).config.integrator_threads, 1);

  auto alternate = RunSettings::instance();
  alternate.default_num_threads = 4;
  RunSettings::init(alternate);
  EXPECT_EQ(ProjectiveIntegrator(inherited).config.num_threads, 4);

  const HydraPipeline pipeline(HydraPipeline::Config{}, context);
  EXPECT_EQ(ProjectiveIntegrator(inherited).config.num_threads, 2);
  RunSettings::init(previous);
}

}  // namespace hydra
