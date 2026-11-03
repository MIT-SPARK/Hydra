#include <gtest/gtest.h>
#include <hydra/backend/merge_event_logger.h>
#include <hydra/frontend/frontend_event_logger.h>
#include <hydra/utils/event_log.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <cmath>
#include <fstream>

#include "hydra_test/shared_dsg_fixture.h"

using namespace spark_dsg;

namespace hydra {
namespace {

std::vector<std::string> readLines(const std::filesystem::path& path) {
  std::ifstream fin(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(fin, line)) {
    lines.push_back(line);
  }
  return lines;
}

struct TempDir {
  TempDir() : path(std::filesystem::temp_directory_path() / "hydra_test_event_log") {
    std::filesystem::remove_all(path);
  }
  ~TempDir() { std::filesystem::remove_all(path); }
  std::filesystem::path path;
};

}  // namespace

TEST(EventLog, AppendsLinesAfterOpenLine) {
  TempDir tmp;
  const auto path = tmp.path / "sub" / "events.jsonl";
  EventLog log(path, "\"robot_id\": 3");
  EXPECT_FALSE(std::filesystem::exists(path));  // created on the first append
  log.append({});
  EXPECT_FALSE(std::filesystem::exists(path));

  log.append({"{\"ev\": \"a\"}", "{\"ev\": \"b\"}"});
  log.append({"{\"ev\": \"c\"}"});
  const auto lines = readLines(path);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(lines[0].rfind("{\"ev\": \"open\", \"version\": 1, \"wall_ns\": ", 0), 0u);
  EXPECT_NE(lines[0].find(", \"robot_id\": 3}"), std::string::npos);
  EXPECT_EQ(lines[1], "{\"ev\": \"a\"}");
  EXPECT_EQ(lines[3], "{\"ev\": \"c\"}");
}

TEST(EventLog, ReopensRemovedFile) {
  TempDir tmp;
  const auto path = tmp.path / "events.jsonl";
  EventLog log(path);
  log.append({"{\"ev\": \"a\"}"});
  std::filesystem::remove_all(tmp.path);  // e.g., an output directory wipe
  log.append({"{\"ev\": \"b\"}"});
  const auto lines = readLines(path);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0].rfind("{\"ev\": \"open\"", 0), 0u);
  EXPECT_EQ(lines[1], "{\"ev\": \"b\"}");
}

TEST(EventLog, AppendsToExistingFile) {
  TempDir tmp;
  const auto path = tmp.path / "events.jsonl";
  EventLog(path).append({"{\"ev\": \"a\"}"});
  EventLog(path).append({"{\"ev\": \"b\"}"});  // e.g., a restarted process
  const auto lines = readLines(path);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(lines[2].rfind("{\"ev\": \"open\"", 0), 0u);
  EXPECT_EQ(lines[3], "{\"ev\": \"b\"}");
}

TEST(FrontendEventLogger, LogsBindingsAndAgents) {
  TempDir tmp;
  FrontendEventLogger::Config config;
  config.path = tmp.path / "frontend.jsonl";
  FrontendEventLogger logger(config);

  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  graph.emplaceNode(
      2,
      NodeSymbol('a', 0),
      std::make_unique<AgentNodeAttributes>(std::chrono::nanoseconds(15),
                                            rot,
                                            Eigen::Vector3d(1.0, 2.0, 3.0),
                                            NodeSymbol('a', 0)),
      'a');

  FrontendOutput output(20, 0);
  output.new_agent_nodes = {NodeSymbol('a', 0), NodeSymbol('a', 1)};  // a1: missing
  TrackBinding bound{4, NodeSymbol('O', 1), 20, 20, std::nullopt};
  TrackBinding ended{3, NodeSymbol('O', 0), 10, 18, std::string("deleted")};
  output.binding_events = {{"OBJECTS", 20, bound}, {"OBJECTS", 20, ended}};
  logger.call(20, graph, output);

  const auto lines = readLines(config.path);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_NE(lines[0].find("\"source\": \"frontend\", \"robot_id\": 0"),
            std::string::npos);
  std::stringstream expected;
  expected << "{\"ev\": \"bind\", \"stamp_ns\": 20, \"layer\": \"OBJECTS\", "
              "\"track_id\": 4, "
           << "\"node_id\": " << static_cast<NodeId>(NodeSymbol('O', 1)) << "}";
  EXPECT_EQ(lines[1], expected.str());
  expected.str("");
  expected
      << "{\"ev\": \"end\", \"stamp_ns\": 20, \"layer\": \"OBJECTS\", \"track_id\": 3, "
      << "\"node_id\": " << static_cast<NodeId>(NodeSymbol('O', 0))
      << ", \"reason\": \"deleted\"}";
  EXPECT_EQ(lines[2], expected.str());
  expected.str("");
  expected << "{\"ev\": \"agent\", \"stamp_ns\": 20, \"node_id\": "
           << static_cast<NodeId>(NodeSymbol('a', 0))
           << ", \"timestamp_ns\": 15, \"world_T_body\": [";
  EXPECT_EQ(lines[3].rfind(expected.str(), 0), 0u) << lines[3];
  // row-major world_T_body: rotation about z by 90 degrees, translation (1, 2, 3)
  std::stringstream values(lines[3].substr(expected.str().size()));
  const std::vector<double> T{0, -1, 0, 1, 1, 0, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1};
  for (size_t i = 0; i < T.size(); ++i) {
    double value;
    char sep;
    values >> value >> sep;
    EXPECT_NEAR(value, T[i], 1.0e-12) << "entry " << i << ": " << lines[3];
  }
}

TEST(FrontendEventLogger, DisabledWithoutPath) {
  FrontendEventLogger logger({});
  auto dsg = test::makeSharedDsg();
  FrontendOutput output(20, 0);
  output.binding_events = {{"OBJECTS", 20, TrackBinding{1, 2, 20, 20, std::nullopt}}};
  logger.call(20, *dsg->graph, output);  // no-op
}

TEST(MergeEventLogger, LogsChangesOnly) {
  TempDir tmp;
  MergeEventLogger::Config config;
  config.path = tmp.path / "backend.jsonl";
  MergeEventLogger logger(config);

  logger.call(10, {});  // nothing to log: no file
  EXPECT_FALSE(std::filesystem::exists(config.path));
  logger.call(20, {{1, 2}});
  logger.call(30, {{1, 2}});          // unchanged
  logger.call(40, {{1, 3}, {2, 3}});  // 2 merged into 3: 1 rewired
  logger.call(50, {{4, 5}});          // reset + new merge

  const auto lines = readLines(config.path);
  ASSERT_EQ(lines.size(), 7u);
  EXPECT_NE(lines[0].find("\"source\": \"backend\""), std::string::npos);
  EXPECT_EQ(lines[1],
            "{\"ev\": \"merge\", \"stamp_ns\": 20, \"child\": 1, \"parent\": 2}");
  EXPECT_EQ(lines[2],
            "{\"ev\": \"merge\", \"stamp_ns\": 40, \"child\": 1, \"parent\": 3}");
  EXPECT_EQ(lines[3],
            "{\"ev\": \"merge\", \"stamp_ns\": 40, \"child\": 2, \"parent\": 3}");
  EXPECT_EQ(lines[4],
            "{\"ev\": \"merge\", \"stamp_ns\": 50, \"child\": 4, \"parent\": 5}");
  EXPECT_EQ(lines[5], "{\"ev\": \"unmerge\", \"stamp_ns\": 50, \"child\": 1}");
  EXPECT_EQ(lines[6], "{\"ev\": \"unmerge\", \"stamp_ns\": 50, \"child\": 2}");
}

TEST(MergeEventLogger, CalledByDsgUpdater) {
  TempDir tmp;
  DsgUpdater::Config config;
  config.merge_sinks.push_back(DsgUpdater::MergeSink::Factory(
      MergeEventLogger::Config{tmp.path / "backend.jsonl"}));
  auto source = test::makeSharedDsg();
  auto target = test::makeSharedDsg();
  DsgUpdater updater(config, source->graph, target);

  target->merges[NodeSymbol('O', 1)] = NodeSymbol('O', 0);
  updater.callUpdateFunctions(10, std::make_shared<UpdateInfo>(UpdateInfo{10}));
  const auto lines = readLines(tmp.path / "backend.jsonl");
  ASSERT_EQ(lines.size(), 2u);
  std::stringstream expected;
  expected << "{\"ev\": \"merge\", \"stamp_ns\": 10, \"child\": "
           << static_cast<NodeId>(NodeSymbol('O', 1))
           << ", \"parent\": " << static_cast<NodeId>(NodeSymbol('O', 0)) << "}";
  EXPECT_EQ(lines[1], expected.str());
}

}  // namespace hydra
