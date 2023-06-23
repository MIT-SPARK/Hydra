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
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra/config/config.h>
#include <hydra/reconstruction/combo_integrator.h>
#include <hydra/reconstruction/configs.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>

#include <Eigen/Dense>

#include "hydra_ros/visualizer/topology_server_visualizer.h"

DEFINE_string(config, "", "gvd integrator yaml config");
DEFINE_string(output_path, "", "output directory");
DEFINE_int32(max_updates, 0, "maximum number of updates per bag");
DEFINE_int32(voxels_per_side, 16, "voxel block size");
DEFINE_double(voxel_size, 0.1, "voxel size");

using config_parser::ConfigVisitor;
using config_parser::YamlParser;
using config_parser::YamlParserImpl;
using hydra::TopologyServerVisualizer;
using hydra::places::ComboIntegrator;
using hydra::places::GvdVoxel;
using sensor_msgs::Image;
using voxblox::BlockIndexList;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfIntegratorBase;
using voxblox::TsdfIntegratorFactory;
using voxblox::TsdfVoxel;

using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
using TimeSync = message_filters::Synchronizer<Policy>;
using TsdfConfig = voxblox::TsdfIntegratorBase::Config;
using MeshConfig = voxblox::MeshIntegratorConfig;
using GvdConfig = hydra::places::GvdIntegratorConfig;

struct ComparisonResult {
  size_t num_missing_lhs = 0;
  size_t num_missing_rhs = 0;
  size_t num_observed = 0;
  size_t num_unobserved = 0;
  size_t num_rhs_seen_lhs_unseen = 0;
  size_t num_lhs_seen_rhs_unseen = 0;
  size_t num_fixed_different = 0;
  size_t num_parent_different = 0;
  size_t num_basis_different = 0;
  size_t num_surface_different = 0;
  double min_error = std::numeric_limits<double>::infinity();
  double max_error = 0.0;
};

std::ostream& operator<<(std::ostream& out, const ComparisonResult& r) {
  return out << "  - " << r.num_missing_lhs << " / " << r.num_missing_rhs
             << " unallocated (lhs / rhs) " << std::endl
             << "  - " << r.num_lhs_seen_rhs_unseen << " / "
             << r.num_rhs_seen_lhs_unseen << " unique (lhs / rhs)" << std::endl
             << "  - observed:   " << r.num_observed << std::endl
             << "  - unobserved: " << r.num_unobserved << std::endl
             << "  - fixed: " << r.num_fixed_different << std::endl
             << "  - parent: " << r.num_parent_different << std::endl
             << "  - basis: " << r.num_basis_different << std::endl
             << "  - surface: " << r.num_surface_different << std::endl
             << "  - error: [" << r.min_error << ", " << r.max_error << "]";
}

size_t getMissingBlocks(const Layer<GvdVoxel>& layer,
                        const BlockIndexList blocks,
                        const Layer<GvdVoxel>& other_layer) {
  size_t num_missing = 0;
  for (const auto& idx : blocks) {
    if (other_layer.hasBlock(idx)) {
      continue;
    }

    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      if (block.getVoxelByLinearIndex(i).observed) {
        num_missing++;
      }
    }
  }
  return num_missing;
}

ComparisonResult compareLayers(const Layer<GvdVoxel>& lhs, const Layer<GvdVoxel>& rhs) {
  voxblox::BlockIndexList lhs_blocks;
  lhs.getAllAllocatedBlocks(&lhs_blocks);

  voxblox::BlockIndexList rhs_blocks;
  rhs.getAllAllocatedBlocks(&rhs_blocks);

  ComparisonResult results;
  results.num_missing_lhs = getMissingBlocks(rhs, rhs_blocks, lhs);
  results.num_missing_rhs = getMissingBlocks(lhs, lhs_blocks, rhs);

  for (const auto& idx : lhs_blocks) {
    if (!rhs.hasBlock(idx)) {
      continue;
    }

    const auto& lhs_block = lhs.getBlockByIndex(idx);
    const auto& rhs_block = rhs.getBlockByIndex(idx);
    for (size_t i = 0; i < lhs_block.num_voxels(); ++i) {
      const GvdVoxel& lhs_voxel = lhs_block.getVoxelByLinearIndex(i);
      const GvdVoxel& rhs_voxel = rhs_block.getVoxelByLinearIndex(i);
      if (!lhs_voxel.observed && !rhs_voxel.observed) {
        results.num_unobserved++;
        continue;
      }

      results.num_observed++;

      if (!lhs_voxel.observed || !rhs_voxel.observed) {
        results.num_rhs_seen_lhs_unseen += (!lhs_voxel.observed ? 1 : 0);
        results.num_lhs_seen_rhs_unseen += (!rhs_voxel.observed ? 1 : 0);
        continue;
      }

      results.num_fixed_different += (lhs_voxel.fixed != rhs_voxel.fixed) ? 1 : 0;
      results.num_parent_different +=
          (lhs_voxel.has_parent != rhs_voxel.has_parent) ? 1 : 0;
      results.num_basis_different +=
          (lhs_voxel.num_extra_basis != rhs_voxel.num_extra_basis) ? 1 : 0;
      results.num_surface_different +=
          (lhs_voxel.on_surface != rhs_voxel.on_surface) ? 1 : 0;

      double error = std::abs(lhs_voxel.distance - rhs_voxel.distance);
      results.min_error = std::min(results.min_error, error);
      results.max_error = std::max(results.max_error, error);
    }
  }

  return results;
}

namespace voxblox {

template <typename Visitor>
void visit_config(const Visitor& v, TsdfIntegratorBase::Config& config) {
  v.visit("default_truncation_distance", config.default_truncation_distance);
  v.visit("max_weight", config.max_weight);
  v.visit("voxel_carving_enabled", config.voxel_carving_enabled);
  v.visit("min_ray_length_m", config.min_ray_length_m);
  v.visit("max_ray_length_m", config.max_ray_length_m);
  v.visit("use_const_weight", config.use_const_weight);
  v.visit("allow_clear", config.allow_clear);
  v.visit("use_weight_dropoff", config.use_weight_dropoff);
  v.visit("use_sparsity_compensation_factor", config.use_sparsity_compensation_factor);
  v.visit("integrator_threads", config.integrator_threads);
  v.visit("integration_order_mode", config.integration_order_mode);
  v.visit("enable_anti_grazing", config.enable_anti_grazing);
  v.visit("start_voxel_subsampling_factor", config.start_voxel_subsampling_factor);
  v.visit("max_consecutive_ray_collisions", config.max_consecutive_ray_collisions);
  v.visit("clear_checks_every_n_frames", config.clear_checks_every_n_frames);
}

}  // namespace voxblox

namespace hydra {

struct BagConfig {
  std::string color_topic;
  std::string depth_topic;
  double start = -1.0;
  double duration = -1.0;
  std::vector<float> intrinsics;
};

template <typename Visitor>
void visit_config(const Visitor& v, BagConfig& config) {
  v.visit("color_topic", config.color_topic);
  v.visit("depth_topic", config.depth_topic);
  v.visit("start", config.start);
  v.visit("duration", config.duration);
  v.visit("intrinsics", config.intrinsics);
}

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, BagConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(voxblox, TsdfIntegratorBase::Config)
using hydra::BagConfig;

bool gvdFlagsSame(const GvdVoxel& lhs, const GvdVoxel& rhs) {
  return lhs.fixed == rhs.fixed && lhs.has_parent == rhs.has_parent &&
         lhs.num_extra_basis == rhs.num_extra_basis && lhs.on_surface == rhs.on_surface;
}

struct GvdValidator {
  struct Intrinstics {
    float fx;
    float fy;
    float cx;
    float cy;
  } intrinsics;

  GvdValidator(const TsdfConfig& tsdf_config,
               const MeshConfig& mesh_config,
               const GvdConfig& gvd_config,
               double voxel_size,
               int voxels_per_side)
      : gvd_config(gvd_config),
        mesh_config(mesh_config),
        voxel_size(voxel_size),
        voxels_per_side(voxels_per_side) {
    // roughly 3000 hours: should be enough to store all the tfs
    ros::Duration max_duration;
    max_duration.fromSec(1.0e7);
    buffer.reset(new tf2::BufferCore(max_duration));

    visualizer.reset(new TopologyServerVisualizer("~"));
    full_visualizer.reset(new TopologyServerVisualizer("~full"));

    tsdf.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
    tsdf_integrator = TsdfIntegratorFactory::create("fast", tsdf_config, tsdf.get());

    gvd.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
    mesh.reset(new MeshLayer(tsdf->block_size()));
    gvd_integrator.reset(
        new ComboIntegrator(gvd_config, tsdf.get(), gvd, mesh, &mesh_config));
  }

  void addTfsFromBag(const rosbag::Bag& bag) {
    std::vector<std::string> tf_topic{"/tf"};
    rosbag::View view(bag, rosbag::TopicQuery(tf_topic));
    for (const auto& m : view) {
      const auto msg = m.instantiate<tf2_msgs::TFMessage>();
      if (!msg) {
        LOG(ERROR) << "Invalid message on /tf topic!";
        continue;
      }

      for (const auto& transform : msg->transforms) {
        buffer->setTransform(transform, "rosbag");
      }
    }

    std::vector<std::string> static_tf_topic{"/tf_static"};
    rosbag::View static_view(bag, rosbag::TopicQuery(static_tf_topic));
    for (const auto& m : static_view) {
      const auto msg = m.instantiate<tf2_msgs::TFMessage>();
      if (!msg) {
        LOG(ERROR) << "Invalid message on /tf topic!";
        continue;
      }

      for (const auto& transform : msg->transforms) {
        buffer->setTransform(transform, "rosbag", true);
      }
    }
  }

  void fillPointCloud(const cv::Mat& color,
                      const cv::Mat& depth,
                      voxblox::Pointcloud& xyz,
                      voxblox::Colors& colors) {
    for (int r = 0; r < color.rows; ++r) {
      for (int c = 0; c < color.cols; ++c) {
        const ssize_t index = r * color.cols + c;
        const float u = (c - intrinsics.cx) / intrinsics.fx;
        const float v = (r - intrinsics.cy) / intrinsics.fy;

        Eigen::Vector3f& point = xyz[index];
        const float depth_value = depth.at<float>(r, c);
        point.x() = u * depth_value;
        point.y() = v * depth_value;
        point.z() = depth_value;

        auto& color_point = colors[index];
        const auto& pixel = color.at<cv::Vec3b>(r, c);
        color_point.r = pixel[0];
        color_point.g = pixel[1];
        color_point.b = pixel[2];
      }
    }
  }

  void handleImages(const sensor_msgs::Image::ConstPtr& rgb_msg,
                    const sensor_msgs::Image::ConstPtr& depth_msg) {
    ++num_updates;

    auto rgb = cv_bridge::toCvCopy(rgb_msg);
    if (rgb->image.empty()) {
      LOG(ERROR) << "invalid rgb image";
      return;
    }

    auto depth_img = cv_bridge::toCvCopy(depth_msg);

    cv::Mat depth;
    if (depth_img->image.type() == CV_32FC1) {
      depth = depth_img->image;
    } else if (depth_img->image.type() == CV_16UC1) {
      depth_img->image.convertTo(depth, CV_32FC1, 1.0e-3);
    } else {
      LOG(FATAL) << "Invalid depth type: " << depth_img->image.type();
      return;
    }

    const size_t total = rgb_msg->width * rgb_msg->height;
    voxblox::Pointcloud xyz(total);
    voxblox::Colors colors(total);
    fillPointCloud(rgb->image, depth, xyz, colors);

    const auto pose = buffer->lookupTransform("world", "left_cam", rgb->header.stamp);

    Eigen::Quaterniond world_q_color;
    tf2::convert(pose.transform.rotation, world_q_color);

    Eigen::Vector3d world_t_color;
    world_t_color << pose.transform.translation.x, pose.transform.translation.y,
        pose.transform.translation.z;

    voxblox::Transformation voxblox_transform(world_q_color.cast<float>(),
                                              world_t_color.cast<float>());
    tsdf_integrator->integratePointCloud(voxblox_transform, xyz, colors, false);

    VLOG(2) << "";
    VLOG(2) << "====================================================================";
    VLOG(2) << "Starting Incremental Update";
    VLOG(2) << "====================================================================";
    gvd_integrator->update(rgb_msg->header.stamp.toNSec(), true, true);

    full_gvd.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
    full_mesh.reset(new MeshLayer(tsdf->block_size()));
    full_gvd_integrator.reset(
        new ComboIntegrator(gvd_config, tsdf.get(), full_gvd, full_mesh, &mesh_config));

    VLOG(2) << "";
    VLOG(2) << "====================================================================";
    VLOG(2) << "Running Full Update" << std::endl;
    VLOG(2) << "====================================================================";
    full_gvd_integrator->update(rgb_msg->header.stamp.toNSec(), false, true);

    auto result = compareLayers(*gvd, *full_gvd);

    LOG(INFO) << std::endl
              << "**********************" << std::endl
              << "* Comparison Results *" << std::endl
              << "**********************" << std::endl
              << result;

    if (visualizer) {
      const uint64_t timestamp_ns = rgb_msg->header.stamp.toNSec();
      visualizer->visualize(gvd_integrator->getGraph(),
                            gvd_integrator->getGvdGraph(),
                            *gvd,
                            *tsdf,
                            timestamp_ns);

      visualizer->visualizeError(*gvd, *full_gvd, 0.0, timestamp_ns);

      voxblox_msgs::Mesh mesh_msg;
      generateVoxbloxMeshMsg(mesh, voxblox::ColorMode::kLambertColor, &mesh_msg);
      mesh_msg.header.frame_id = "world";
      mesh_msg.header.stamp = rgb_msg->header.stamp;
      mesh_pub.publish(mesh_msg);

      ros::spinOnce();
    }

    if (full_visualizer) {
      const uint64_t timestamp_ns = rgb_msg->header.stamp.toNSec();
      full_visualizer->visualize(full_gvd_integrator->getGraph(),
                                 full_gvd_integrator->getGvdGraph(),
                                 *full_gvd,
                                 *tsdf,
                                 timestamp_ns);
      ros::spinOnce();
    }
  }

  void readBag(const std::string& bag_path, const BagConfig& config) {
    rosbag::Bag bag;
    LOG(INFO) << "Opening " << bag_path << " ...";
    bag.open(bag_path, rosbag::bagmode::Read);
    LOG(INFO) << "Opened " << bag_path;
    LOG(INFO) << "Adding bag tfs to buffer...";
    addTfsFromBag(bag);
    LOG(INFO) << "Added bag tfs to buffer";

    std::vector<std::string> topics{config.color_topic, config.depth_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    bool have_start = false;
    ros::Time start;

    TimeSync sync(Policy(10));
    sync.registerCallback(&GvdValidator::handleImages, this);

    for (const auto& m : view) {
      if (visualizer && !ros::ok()) {
        break;
      }

      const auto topic = m.getTopic();
      if (!have_start) {
        start = m.getTime();
        if (config.start >= 0.0) {
          start += ros::Duration(config.start);
        }
        have_start = true;
      }

      const auto diff_s = (m.getTime() - start).toSec();
      if (diff_s < 0.0) {
        VLOG(2) << "Skipping message " << std::abs(diff_s) << " [s] before start";
        continue;
      }

      if (config.duration >= 0.0 && diff_s > config.duration) {
        LOG(INFO) << "Reached end of duration: " << diff_s << " [s]";
        return;
      }

      auto msg = m.instantiate<sensor_msgs::Image>();
      if (!msg) {
        LOG(ERROR) << "unable to get image from " << topic;
        return;
      }

      if (topic == config.color_topic) {
        sync.add<0>(ros::MessageEvent<Image>(msg, m.getTime()));
      } else {
        sync.add<1>(ros::MessageEvent<Image>(msg, m.getTime()));
      }

      if (FLAGS_max_updates > 0 && num_updates >= FLAGS_max_updates) {
        break;
      }
    }

    bag.close();
  }

  GvdConfig gvd_config;
  MeshConfig mesh_config;
  double voxel_size;
  int voxels_per_side;

  int64_t num_updates = 0;
  std::unique_ptr<tf2::BufferCore> buffer;

  Layer<TsdfVoxel>::Ptr tsdf;
  TsdfIntegratorBase::Ptr tsdf_integrator;

  Layer<GvdVoxel>::Ptr gvd;
  MeshLayer::Ptr mesh;
  std::unique_ptr<ComboIntegrator> gvd_integrator;
  std::unique_ptr<TopologyServerVisualizer> visualizer;

  Layer<GvdVoxel>::Ptr full_gvd;
  MeshLayer::Ptr full_mesh;
  std::unique_ptr<ComboIntegrator> full_gvd_integrator;
  std::unique_ptr<TopologyServerVisualizer> full_visualizer;

  ros::Publisher mesh_pub;
};

struct ValidatorLogger : public config_parser::Logger {
  void log_missing(const std::string& message) const override { LOG(ERROR) << message; }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gvd_validator");
  ros::NodeHandle nh("");

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_v = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc <= 1) {
    LOG(FATAL) << "Missing required bag path";
    return 1;
  }

  const std::string bag_path(argv[1]);

  YAML::Node node = YAML::LoadFile(FLAGS_config);

  YamlParser parser(std::make_unique<YamlParserImpl>(node),
                    std::make_shared<ValidatorLogger>());

  BagConfig config;
  ConfigVisitor<BagConfig>::visit_config(parser, config);

  TsdfConfig tsdf_config;
  ConfigVisitor<TsdfConfig>::visit_config(parser, tsdf_config);

  MeshConfig mesh_config;
  ConfigVisitor<MeshConfig>::visit_config(parser, mesh_config);

  GvdConfig gvd_config;
  ConfigVisitor<GvdConfig>::visit_config(parser, gvd_config);

  VLOG(1) << "BagConfig:" << std::endl << config;
  VLOG(1) << "TsdfConfig:" << std::endl << tsdf_config;
  VLOG(1) << "MeshConfig:" << std::endl << mesh_config;
  VLOG(1) << "GvdConfig:" << std::endl << gvd_config;

  GvdValidator validator(
      tsdf_config, mesh_config, gvd_config, FLAGS_voxel_size, FLAGS_voxels_per_side);
  validator.mesh_pub = nh.advertise<voxblox_msgs::Mesh>("mesh_viz", 1, true);
  validator.intrinsics.fx = config.intrinsics.at(0);
  validator.intrinsics.fy = config.intrinsics.at(1);
  validator.intrinsics.cx = config.intrinsics.at(2);
  validator.intrinsics.cy = config.intrinsics.at(3);
  validator.readBag(bag_path, config);
  return 0;
}
