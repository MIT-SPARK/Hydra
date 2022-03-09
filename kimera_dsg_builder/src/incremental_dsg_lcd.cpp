#include "kimera_dsg_builder/incremental_dsg_lcd.h"

#include <glog/logging.h>
#include <hydra_utils/timing_utilities.h>
#include <kimera_pgmo/utils/CommonFunctions.h>

namespace kimera {
namespace incremental {

using hydra::timing::ScopedTimer;
using lcd::LayerRegistrationConfig;

DsgLcd::DsgLcd(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg)
    : nh_(nh), dsg_(dsg), lcd_graph_(new DynamicSceneGraph()) {
  // TODO(nathan) rethink
  int robot_id = 0;
  nh_.getParam("robot_id", robot_id);
  robot_prefix_ = kimera_pgmo::robot_id_to_prefix.at(robot_id);

  config_ = load_config<lcd::DsgLcdConfig>(nh_, "lcd");
  // TODO(nathan) think about fixing lcd log path

  config_.agent_search_config.min_registration_score =
      config_.agent_search_config.min_score;
  lcd_module_.reset(new lcd::DsgLcdModule(config_));
}

void DsgLcd::stop() {
  VLOG(2) << "[DSG LCD] stopping lcd!";

  should_shutdown_ = true;
  if (lcd_thread_) {
    VLOG(2) << "[DSG LCD] joining thread";
    lcd_thread_->join();
    lcd_thread_.reset();
    VLOG(2) << "[DSG LCD] joined thread";
  }

  lcd_visualizer_.reset();
}

DsgLcd::~DsgLcd() { stop(); }

void DsgLcd::handleDbowMsg(const kimera_vio_ros::BowQuery::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  bow_messages_.push_back(msg);
}

void DsgLcd::start() {
  bow_sub_ = nh_.subscribe("bow_vectors", 100, &DsgLcd::handleDbowMsg, this);

  if (config_.visualize_dsg_lcd) {
    ros::NodeHandle nh(config_.lcd_visualizer_ns);
    visualizer_queue_.reset(new ros::CallbackQueue());
    nh.setCallbackQueue(visualizer_queue_.get());

    lcd_visualizer_.reset(new lcd::LcdVisualizer(nh, config_.object_radius_m));
    lcd_visualizer_->setGraph(lcd_graph_);
    lcd_visualizer_->setLcdModule(lcd_module_.get());
  }

  lcd_thread_.reset(new std::thread(&DsgLcd::runLcd, this));
  LOG(INFO) << "[DSG LCD] LCD started!";
}

std::optional<NodeId> DsgLcd::getLatestAgentId() {
  if (lcd_queue_.empty()) {
    return std::nullopt;
  }

  const auto& node = lcd_graph_->getDynamicNode(lcd_queue_.top())->get();
  const auto prev_time = node.timestamp;
  const bool has_parent = node.hasParent();

  if (!has_parent) {
    LOG(ERROR) << "Found agent node without parent: "
               << NodeSymbol(lcd_queue_.top()).getLabel() << ". Discarding!";
    lcd_queue_.pop();
    return std::nullopt;
  }

  const std::chrono::nanoseconds curr_time(last_places_timestamp_);
  std::chrono::duration<double> diff_s = curr_time - prev_time;
  // we consider should_shutdown_ here to make sure we're not waiting on popping from
  // the LCD queue while not getting new place messages
  if (!should_shutdown_ && diff_s.count() < config_.lcd_agent_horizon_s) {
    return std::nullopt;
  }

  const bool forced_pop = (diff_s.count() < config_.lcd_agent_horizon_s);
  if (should_shutdown_ && forced_pop) {
    LOG(ERROR) << "Forcing pop of node " << NodeSymbol(lcd_queue_.top()).getLabel()
               << " from lcd queue due to shutdown: parent? "
               << (has_parent ? "yes" : "no") << ", diff: " << diff_s.count() << " / "
               << config_.lcd_agent_horizon_s;
  }

  auto valid_node = lcd_queue_.top();
  lcd_queue_.pop();
  return valid_node;
}

void DsgLcd::runLcd() {
  ros::WallRate r(10);
  while (ros::ok()) {
    assignBowVectors();

    {  // start critical section
      std::unique_lock<std::mutex> lock(dsg_->mutex);
      lcd_graph_->mergeGraph(*dsg_->graph);
    }  // end critical section

    if (lcd_graph_->getLayer(KimeraDsgLayers::PLACES).numNodes() == 0) {
      r.sleep();
      continue;
    }

    if (should_shutdown_ && lcd_queue_.empty()) {
      break;
    }

    {  // start critical section
      std::unique_lock<std::mutex> place_lock(places_queue_mutex_);
      potential_lcd_root_nodes_.insert(potential_lcd_root_nodes_.end(),
                                       archived_places_.begin(),
                                       archived_places_.end());
      archived_places_.clear();
    }  // end critical section

    auto latest_agent = getLatestAgentId();
    if (!latest_agent) {
      r.sleep();
      continue;
    }

    const Eigen::Vector3d latest_pos = lcd_graph_->getPosition(*latest_agent);

    NodeIdSet to_cache;
    auto iter = potential_lcd_root_nodes_.begin();
    while (iter != potential_lcd_root_nodes_.end()) {
      const Eigen::Vector3d pos = lcd_graph_->getPosition(*iter);
      if ((latest_pos - pos).norm() < config_.descriptor_creation_horizon_m) {
        ++iter;
      } else {
        to_cache.insert(*iter);
        iter = potential_lcd_root_nodes_.erase(iter);
      }
    }

    if (!to_cache.empty()) {
      auto curr_time = ros::Time::now();
      lcd_module_->updateDescriptorCache(*lcd_graph_, to_cache, curr_time.toNSec());
    }

    auto time = lcd_graph_->getDynamicNode(*latest_agent).value().get().timestamp;
    auto results = lcd_module_->detect(*lcd_graph_, *latest_agent, time.count());
    if (lcd_visualizer_) {
      lcd_visualizer_->setGraphUpdated();
      lcd_visualizer_->redraw();
    }

    if (results.size() == 0) {
      if (!should_shutdown_) {
        r.sleep();
      }
      continue;
    }

    {  // start lcd critical section
      // TODO(nathan) double check logic here
      std::unique_lock<std::mutex> lcd_lock(dsg_->lcd_mutex);
      for (const auto& result : results) {
        dsg_->loop_closures.push(result);
        LOG(WARNING) << "Found valid loop-closure: "
                     << NodeSymbol(result.from_node).getLabel() << " -> "
                     << NodeSymbol(result.to_node).getLabel();
      }
    }  // end lcd critical section

    if (!should_shutdown_) {
      r.sleep();
    }
  }
}

void DsgLcd::assignBowVectors() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& agents = dsg_->graph->getLayer(KimeraDsgLayers::AGENTS, robot_prefix_);

  const size_t prior_size = bow_messages_.size();
  auto iter = bow_messages_.begin();
  while (iter != bow_messages_.end()) {
    // TODO(nathan) implicit assumption that gtsam symbol and dsg node symbol are same
    const auto& msg = *iter;
    char prefix = kimera_pgmo::robot_id_to_prefix.at(msg->robot_id);
    NodeSymbol pgmo_key(prefix, msg->pose_id);
    if (agent_key_map_.count(pgmo_key)) {
      const auto& node = agents.getNodeByIndex(agent_key_map_.at(pgmo_key))->get();
      lcd_queue_.push(node.id);

      AgentNodeAttributes& attrs = node.attributes<AgentNodeAttributes>();
      attrs.dbow_ids = Eigen::Map<const AgentNodeAttributes::BowIdVector>(
          msg->bow_vector.word_ids.data(), msg->bow_vector.word_ids.size());
      attrs.dbow_values = Eigen::Map<const Eigen::VectorXf>(
          msg->bow_vector.word_values.data(), msg->bow_vector.word_values.size());

      iter = bow_messages_.erase(iter);
    } else {
      ++iter;
    }
  }

  VLOG(3) << "[DSG LCD] " << bow_messages_.size() << " of " << prior_size
          << " bow vectors unassigned";
}

}  // namespace incremental
}  // namespace kimera
