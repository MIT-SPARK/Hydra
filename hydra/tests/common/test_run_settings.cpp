#include <config_utilities/parsing/yaml.h>
#include <gtest/gtest.h>

#include "hydra/common/graph_update.h"
#include "hydra/common/hydra_pipeline.h"
#include "hydra/common/run_settings.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"

namespace hydra {

TEST(RunSettings, DisabledDiagnosticsDoNotEvaluateArguments) {
  const VerbosityConfig silent("silent", -1);
  auto evaluations = 0;
  const auto expensive = [&] {
    ++evaluations;
    return "diagnostic";
  };
  MLOG_WITH(silent, 0, "{}", expensive());
  EXPECT_EQ(evaluations, 0);
}

TEST(RunSettings, FatalChecksRemainActive) {
  auto evaluations = 0;
  const auto value = [&] {
    ++evaluations;
    return evaluations;
  };
  HYDRA_CHECK_EQ(value(), 1);
  EXPECT_EQ(evaluations, 1);
  EXPECT_DEATH(HYDRA_CHECK_EQ(1, 2), "");
  EXPECT_DEATH(HYDRA_LOG(FATAL, "Unrecoverable invariant failure"), "");
}

TEST(RunSettings, VerbosityFollowsActiveRunUnlessOverridden) {
  const auto previous = RunSettings::instance();
  VerbosityConfig inherited;
  VerbosityConfig explicit_zero("fixed", 0);
  RunContext::Config first;
  first.settings.default_verbosity = 2;
  const auto context = std::make_shared<RunContext>(first);
  EXPECT_EQ(inherited.get_verbosity(), 2);
  EXPECT_EQ(explicit_zero.get_verbosity(), 0);

  RunContext::Config second;
  second.settings.default_verbosity = 5;
  const RunContext next(second);
  EXPECT_EQ(inherited.get_verbosity(), 5);
  EXPECT_FALSE(explicit_zero.enabled(1));

  const HydraPipeline pipeline(HydraPipeline::Config{}, context);
  EXPECT_EQ(inherited.get_verbosity(), 2);
  EXPECT_EQ(explicit_zero.get_verbosity(), 0);
  RunSettings::init(previous);
}

TEST(RunSettings, LoggingLabelDoesNotChangeNodeSymbols) {
  const VerbosityConfig defaults("objects");
  EXPECT_EQ(defaults.get_log_prefix(), "[objects] ");
  EXPECT_TRUE(VerbosityConfig{}.get_log_prefix().empty());

  const auto node = YAML::Load(
      "{prefix: Q, log_name: objects, log_prefix: '<', "
      "log_suffix: '> ', verbosity: 3}");
  const auto config = config::fromYaml<LayerTracker::Config>(node);
  EXPECT_EQ(config.prefix, 'Q');
  EXPECT_EQ(config.get_log_prefix(), "<objects> ");
  EXPECT_EQ(config.get_verbosity(), 3);
  const LayerTracker tracker(config);
  EXPECT_EQ(tracker.next_id.category(), 'Q');

  const auto renamed = config.with_name("places");
  EXPECT_EQ(renamed.get_log_prefix(), "<places> ");
  EXPECT_EQ(renamed.get_verbosity(), 3);
  EXPECT_EQ(config.get_log_prefix(), "<objects> ");
  EXPECT_TRUE(config.with_name("").get_log_prefix().empty());
}

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
