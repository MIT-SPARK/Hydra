#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/input/sensor_extrinsics.h>
#include <hydra/utils/pgmo_glog_sink.h>
#include <hydra_multi/common/multi_global_info.h>
#include <hydra_multi/common/multi_pipeline.h>
#include <hydra_multi/input/file_dgraph_input.h>
#include <hydra_multi/input/file_dsg_input.h>
#include <spark_dsg/serialization/json_conversions.h>

#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace hydra_multi {

namespace fs = std::filesystem;

using ExtrinsicsMap = std::map<RobotId, Eigen::Isometry3d>;
using hydra::SensorExtrinsics;
using pose_graph_tools::PoseGraph;

namespace {

struct DataLoopClosure {
  RobotId robot_from;
  uint64_t time_from;
  RobotId robot_to;
  uint64_t time_to;
  Eigen::Vector3d to_p_from;
  Eigen::Quaterniond to_R_from;
  std::optional<double> variance;
  bool in_body_frame = true;

  inline Eigen::Isometry3d to_T_from() const {
    return Eigen::Translation3d(to_p_from) * to_R_from;
  }
};

void to_json(const DataLoopClosure& loop_closure, nlohmann::json& record) {
  record["robot_from"] = loop_closure.robot_from;
  record["time_from"] = loop_closure.time_from;
  record["robot_to"] = loop_closure.robot_to;
  record["time_to"] = loop_closure.time_to;
  record["to_p_from"] = loop_closure.to_p_from;
  record["to_R_from"] = loop_closure.to_R_from;
  if (loop_closure.variance) {
    record["variance"] = loop_closure.variance.value();
  }
  record["in_body_frame"] = loop_closure.in_body_frame;
}

void from_json(const nlohmann::json& record, DataLoopClosure& loop_closure) {
  record.at("robot_from").get_to(loop_closure.robot_from);
  record.at("time_from").get_to(loop_closure.time_from);
  record.at("robot_to").get_to(loop_closure.robot_to);
  record.at("time_to").get_to(loop_closure.time_to);
  record.at("to_p_from").get_to(loop_closure.to_p_from);
  record.at("to_R_from").get_to(loop_closure.to_R_from);
  if (record.count("variance")) {
    loop_closure.variance = record.at("variance").get<double>();
  }

  if (record.count("in_body_frame")) {
    record.at("in_body_frame").get_to(loop_closure.in_body_frame);
  } else {
    loop_closure.in_body_frame = true;
  }
}

inline Eigen::Isometry3d loadExtrinsicsFromYAML(const fs::path& config_path) {
  const auto node = YAML::LoadFile(config_path);
  auto iter = node["sensors"].begin();
  if (iter == node["sensors"].end()) {
    return Eigen::Isometry3d::Identity();
  }

  auto extrinsics =
      config::createFromYaml<SensorExtrinsics>(iter->second["extrinsics"]);
  if (extrinsics) {
    return *extrinsics;
  } else {
    return Eigen::Isometry3d::Identity();
  }
}

inline Eigen::Isometry3d lookupExtrinsics(const ExtrinsicsMap& extrinsics,
                                          RobotId robot) {
  auto iter = extrinsics.find(robot);
  CHECK(iter != extrinsics.end()) << "Missing required extrinsics for robot " << robot;
  return iter->second;
}

PoseGraph loadLoopClosures(const ExtrinsicsMap& extrinsics, const fs::path& lcd_path) {
  PoseGraph loop_closures;
  std::ifstream fin(lcd_path);
  const auto contents = nlohmann::json::parse(fin);
  const auto data = contents.get<std::vector<DataLoopClosure>>();
  for (const auto& lc : data) {
    nlohmann::json record;
    to_json(lc, record);
    VLOG(5) << "Processing " << record;

    auto& edge = loop_closures.edges.emplace_back();
    edge.robot_from = lc.robot_from;
    edge.robot_to = lc.robot_to;
    edge.key_from = lc.time_from;
    edge.key_to = lc.time_to;

    Eigen::Isometry3d bt_T_bf;
    if (lc.in_body_frame) {
      bt_T_bf = lc.to_T_from();
    } else {
      const auto bf_T_sf = lookupExtrinsics(extrinsics, lc.robot_from);
      const auto bt_T_st = lookupExtrinsics(extrinsics, lc.robot_to);
      const auto st_T_sf = lc.to_T_from();
      bt_T_bf = bt_T_st * st_T_sf * bf_T_sf.inverse();
    }

    edge.pose = bt_T_bf;
  }

  return loop_closures;
}

}  // namespace

struct ProblemInfo {
  struct Args {
    int log_level = 0;
    int verbosity = 0;
    int starting_robot_id = 0;
    std::vector<fs::path> inputs;
    fs::path lcd_path;
    fs::path output;

    void add_to_app(CLI::App& app);
  };

  explicit ProblemInfo(const Args& args);
  MultiPipelineConfig config() const;

  PoseGraph loop_closures;
  ExtrinsicsMap extrinsics;
  hydra::DataDirectory output;
  std::vector<config::VirtualConfig<UnitInterface>> robots;
};

void ProblemInfo::Args::add_to_app(CLI::App& app) {
  app.add_option("--verbosity", verbosity)->description("Verbosity for glog");
  app.add_option("--log-level", log_level)->description("Log level for glog");
  app.add_option("-s,--starting-robot-id", starting_robot_id)
      ->description("Robot ID to assign to first input");
  app.add_option("inputs", inputs)
      ->check(CLI::ExistingDirectory)
      ->description("Input files");
  app.add_option("-l,--loop-closures", lcd_path)
      ->check(CLI::ExistingFile)
      ->description("Input loop closures");
  app.add_option("-o,--output", output)->description("Output path");
}

ProblemInfo::ProblemInfo(const Args& args)
    : output(args.output.empty() ? hydra::DataDirectory{}
                                 : hydra::DataDirectory{args.output}) {
  size_t robot_id = args.starting_robot_id;
  for (const auto& input_path : args.inputs) {
    const auto dirpath = fs::absolute(input_path);
    const auto config_path = dirpath / "hydra_config.yaml";
    if (!fs::exists(config_path)) {
      LOG(WARNING) << "Missing hydra config @ " << config_path;
    } else {
      extrinsics[robot_id] = loadExtrinsicsFromYAML(config_path);
    }

    LOG(INFO) << "Merging " << dirpath << " with ID " << robot_id << std::endl;
    hydra_multi::UnitInterface::Config robot_config;
    robot_config.robot_id = robot_id;
    robot_config.robot_name = dirpath.stem();
    ++robot_id;

    // TODO(nathan) configure mesh
    hydra_multi::FileDsgInput::Config dsg;
    dsg.dsg_json = dirpath / "frontend/dsg_with_mesh.json";
    dsg.force_robot_id = true;
    robot_config.inputs.emplace_back("dsg", dsg);

    hydra_multi::FileDGraphInput::Config dgraph;
    dgraph.dgrf_path = dirpath / "backend/deformation_graph.dgrf";
    dgraph.include_priors = false;
    robot_config.inputs.emplace_back("dgrf", dgraph);

    robots.push_back(config::VirtualConfig<UnitInterface>(robot_config));
  }

  if (!args.lcd_path.empty()) {
    loop_closures = loadLoopClosures(extrinsics, args.lcd_path);
  }
}

MultiPipelineConfig ProblemInfo::config() const {
  auto config = config::fromContext<MultiPipelineConfig>();
  config.enable_pgmo_logging = false;
  config.robots = robots;
  return config;
}

class MultiOfflinePipeline : public MultiPipeline {
 public:
  MultiOfflinePipeline(const MultiPipelineConfig& config)
      : MultiPipeline(config::checkValid(config)) {}

  void fuse(const PoseGraph& loop_closures = {}) {
    backend_->spinOnce(0);
    if (loop_closures.empty()) {
      return;
    }

    backend_->addLoopClosures(loop_closures);
    backend_->optimizeOnce(0);
    backend_->spinOnce(0);
  }
};

}  // namespace hydra_multi

int main(int argc, char* argv[]) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();

  CLI::App app("Utility to fuse multiple scene graphs");
  app.allow_extras();
  app.get_formatter()->column_width(50);

  hydra_multi::ProblemInfo::Args args;
  args.add_to_app(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  FLAGS_minloglevel = args.log_level;
  FLAGS_v = args.verbosity;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  logging::Logger::addSink("glog", std::make_shared<hydra::PgmoGlogSink>(5));

  const hydra_multi::ProblemInfo problem(args);
  if (problem.robots.empty()) {
    return 0;
  }

  hydra_multi::MultiOfflinePipeline multi_offline(problem.config());
  multi_offline.init();
  multi_offline.fuse(problem.loop_closures);
  if (problem.output) {
    multi_offline.save(problem.output);
  }

  return 0;
}
