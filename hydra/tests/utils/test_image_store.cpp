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
#include <config_utilities/parsing/yaml.h>
#include <gtest/gtest.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/image_store.h>
#include <spark_dsg/node_symbol.h>
#include <sqlite3.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <thread>

#include "hydra_test/config_guard.h"
#include "hydra_test/image_store_query.h"
#include "hydra_test/temp_directory.h"

namespace hydra {
namespace {

namespace fs = std::filesystem;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;

CameraCalib makeCalib() {
  CameraCalib calib;
  calib.fx = 4.0;
  calib.fy = 5.0;
  calib.cx = 4.0;
  calib.cy = 2.0;
  calib.width = 8;
  calib.height = 4;
  calib.body_T_sensor.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
  return calib;
}

cv::Mat makeColor() { return cv::Mat(4, 8, CV_8UC3, cv::Scalar(10, 20, 30)); }

cv::Mat makeDepth() { return cv::Mat(4, 8, CV_32FC1, cv::Scalar(1.5f)); }

ImageStore::Frame makeFrame(uint64_t sensor_id,
                            uint64_t stamp_ns,
                            ImageStore::FrameKind kind,
                            int robot_id = 0) {
  ImageStore::Frame frame;
  frame.robot_id = robot_id;
  frame.sensor_id = sensor_id;
  frame.timestamp_ns = stamp_ns;
  frame.kind = kind;
  frame.world_T_body.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  return frame;
}

std::string id(uint64_t value) { return std::to_string(static_cast<int64_t>(value)); }

std::string count(const fs::path& dir, const std::string& table) {
  return test::queryValue(dir, "SELECT COUNT(*) FROM " + table);
}

std::string readFile(const fs::path& path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void writeFile(const fs::path& path, const std::string& contents) {
  fs::create_directories(path.parent_path());
  std::ofstream(path) << contents;
}

size_t numEntries(const fs::path& dir) {
  if (!fs::exists(dir)) {
    return 0;
  }

  return std::distance(fs::directory_iterator(dir), fs::directory_iterator());
}

// Leave a store at `path` as an earlier process would: it is written under a directory
// that was never opened at `path` in this process and moved into place afterwards
void makeEarlierSession(const fs::path& path, uint64_t num_frames) {
  static size_t index = 0;
  const auto staging = path.parent_path() / ("staging_" + std::to_string(index++) +
                                             "_" + path.filename().string());
  {
    ImageStore::Config config;
    config.path = staging;
    auto store = ImageStore::open(config);
    ASSERT_TRUE(store);
    const auto sensor = store->addSensor(0, "camera", makeCalib());
    for (uint64_t i = 0; i < num_frames; ++i) {
      auto frame = makeFrame(sensor, 100 * (i + 1), ImageStore::FrameKind::Keyframe);
      frame.anchor_node = NodeSymbol('a', i);
      store->addFrame(frame, makeColor());
    }

    store->bindTrack(0, 1, NodeSymbol('O', 1));
    store->recordMerges({{NodeSymbol('O', 2), NodeSymbol('O', 1)}});
  }

  fs::create_directories(path.parent_path());
  fs::rename(staging, path);
}

//! Open a store that rotates an earlier session
ImageStore::Ptr openRotating(const fs::path& path) {
  ImageStore::Config config;
  config.path = path;
  config.on_existing = ImageStore::OnExisting::Rotate;
  return ImageStore::open(config);
}

}  // namespace

TEST(ImageStore, MaskRle) {
  cv::Mat mask = cv::Mat::zeros(2, 3, CV_8UC1);
  EXPECT_EQ(ImageStore::encodeMaskRle(mask), "[6]");
  mask.at<uint8_t>(0, 1) = 255;
  mask.at<uint8_t>(0, 2) = 1;
  mask.at<uint8_t>(1, 2) = 255;
  EXPECT_EQ(ImageStore::encodeMaskRle(mask), "[1,2,2,1]");
  // masks starting with the object start with an empty background run
  mask.at<uint8_t>(0, 0) = 255;
  EXPECT_EQ(ImageStore::encodeMaskRle(mask), "[0,3,2,1]");
  EXPECT_EQ(ImageStore::encodeMaskRle(cv::Mat()), "[]");
}

TEST(ImageStore, IdsUseRobotPrefix) {
  EXPECT_EQ(ImageStore::makeId(0, 5), 5u);
  EXPECT_EQ(ImageStore::makeId(2, 5), (uint64_t(2) << 48) | 5);
}

// The pipeline config holds the single store of all modules
TEST(ImageStore, GlobalInfoOpensConfiguredStore) {
  test::TempDirectory tmp;
  {
    const auto guard = test::ConfigGuard();
    EXPECT_FALSE(GlobalInfo::instance().imageStore());
  }

  const auto node = YAML::Load("{image_store: {path: " + (tmp.path / "store").string() +
                               ", on_existing: rotate, jpeg_quality: 80}}");
  const auto config = config::fromYaml<PipelineConfig>(node);
  EXPECT_EQ(config.image_store.path, tmp.path / "store");
  EXPECT_EQ(config.image_store.on_existing, ImageStore::OnExisting::Rotate);
  EXPECT_EQ(config.image_store.jpeg_quality, 80);
  EXPECT_EQ(ImageStore::Config().on_existing, ImageStore::OnExisting::Append);

  const test::ConfigGuard guard(false);
  GlobalInfo::init(config);
  const auto store = GlobalInfo::instance().imageStore();
  ASSERT_TRUE(store);
  EXPECT_EQ(store.get(), GlobalInfo::instance().imageStore().get());
  EXPECT_EQ(store->config.jpeg_quality, 80);
  EXPECT_TRUE(fs::exists(tmp.path / "store" / ImageStore::kDatabaseName));
}

// One instance is shared per directory
TEST(ImageStore, SharedInstancePerDirectory) {
  test::TempDirectory tmp;
  EXPECT_FALSE(ImageStore::open(fs::path()));
  const auto first = ImageStore::open(tmp.path / "store");
  ASSERT_TRUE(first);
  const auto second = ImageStore::open(tmp.path / "store/");
  const auto third = ImageStore::open(tmp.path / "store/../store");
  EXPECT_EQ(first.get(), second.get());
  EXPECT_EQ(first.get(), third.get());
  EXPECT_NE(first.get(), ImageStore::open(tmp.path / "other").get());
}

// Everything written can be read back from the database and files
TEST(ImageStore, SchemaRoundTrip) {
  test::TempDirectory tmp;
  const auto dir = tmp.path / "store";
  const int robot = 1;
  const NodeId agent = NodeSymbol('a', 3);
  const NodeId object = NodeSymbol('O', 4);
  uint64_t sensor_id, keyframe_id, obs_frame_id;
  std::vector<uint64_t> obs_ids;
  {
    auto store = ImageStore::open(dir);
    ASSERT_TRUE(store);
    sensor_id = store->addSensor(robot, "camera", makeCalib());
    EXPECT_EQ(sensor_id, ImageStore::makeId(robot, 0));

    auto frame = makeFrame(sensor_id, 1000, ImageStore::FrameKind::Keyframe, robot);
    frame.anchor_node = agent;
    keyframe_id = store->addFrame(frame, makeColor(), makeDepth());
    EXPECT_EQ(keyframe_id, ImageStore::makeId(robot, 0));

    obs_frame_id = store->addFrame(
        makeFrame(sensor_id, 2000, ImageStore::FrameKind::Observation, robot),
        makeColor());

    cv::Mat mask = cv::Mat::zeros(4, 8, CV_8UC1);
    mask(cv::Rect(2, 1, 3, 2)).setTo(255);
    ImageStore::Observation kept;
    kept.frame_id = obs_frame_id;
    kept.timestamp_ns = 2000;
    kept.bbox2d = std::array<int, 4>{2, 1, 4, 2};
    kept.bbox3d = {Eigen::Vector3d(-1, -2, -3), Eigen::Vector3d(1, 2, 3)};
    kept.mask_rle = ImageStore::encodeMaskRle(mask);
    kept.score = 2.5;
    kept.kept = true;
    kept.mask = mask;
    ImageStore::Observation other;
    other.frame_id = keyframe_id;
    other.timestamp_ns = 1000;
    other.score = 1.0;
    obs_ids = store->addObservations(robot, 12, {kept, other});
    ASSERT_EQ(obs_ids.size(), 2u);
    EXPECT_EQ(obs_ids[0], ImageStore::makeId(robot, 0));
    EXPECT_EQ(obs_ids[1], ImageStore::makeId(robot, 1));

    store->bindTrack(robot, 12, object);
    store->recordMerges({{NodeSymbol('O', 5), object}});
    store->flush();
  }

  EXPECT_EQ(
      test::queryValue(dir, "SELECT value FROM meta WHERE key = 'schema_version'"),
      "1");
  EXPECT_EQ(test::queryValue(dir, "PRAGMA journal_mode"), "wal");

  auto rows = test::queryStore(
      dir,
      "SELECT sensor_id, robot_id, name, width, height, fx, fy, cx, cy, depth_scale, "
      "length(body_T_sensor) FROM sensors");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0],
            (std::vector<std::string>{id(sensor_id),
                                      "1",
                                      "camera",
                                      "8",
                                      "4",
                                      "4.0",
                                      "5.0",
                                      "4.0",
                                      "2.0",
                                      "0.001",
                                      "128"}));

  // body_T_sensor is row-major: the translation is the last column
  sqlite3* db = nullptr;
  ASSERT_EQ(sqlite3_open_v2(
                (dir / "store.sqlite").c_str(), &db, SQLITE_OPEN_READONLY, nullptr),
            SQLITE_OK);
  sqlite3_stmt* stmt = nullptr;
  ASSERT_EQ(
      sqlite3_prepare_v2(db, "SELECT body_T_sensor FROM sensors", -1, &stmt, nullptr),
      SQLITE_OK);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  const auto values = static_cast<const double*>(sqlite3_column_blob(stmt, 0));
  EXPECT_DOUBLE_EQ(values[3], 0.1);
  EXPECT_DOUBLE_EQ(values[7], 0.2);
  EXPECT_DOUBLE_EQ(values[11], 0.3);
  EXPECT_DOUBLE_EQ(values[15], 1.0);
  sqlite3_finalize(stmt);
  sqlite3_close(db);

  rows = test::queryStore(dir,
                          "SELECT frame_id, robot_id, sensor_id, timestamp_ns, kind, "
                          "length(world_T_body_odom), anchor_node_id, "
                          "length(anchor_T_body), rgb_path, depth_path FROM frames "
                          "ORDER BY timestamp_ns");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0],
            (std::vector<std::string>{id(keyframe_id),
                                      "1",
                                      id(sensor_id),
                                      "1000",
                                      "0",
                                      "128",
                                      id(agent),
                                      "128",
                                      "frames/r1/1000_rgb.jpg",
                                      "frames/r1/1000_depth.png"}));
  EXPECT_EQ(rows[1],
            (std::vector<std::string>{id(obs_frame_id),
                                      "1",
                                      id(sensor_id),
                                      "2000",
                                      "2",
                                      "128",
                                      "NULL",
                                      "NULL",
                                      "frames/r1/2000_rgb.jpg",
                                      "NULL"}));
  EXPECT_TRUE(fs::exists(dir / "frames/r1/1000_rgb.jpg"));
  EXPECT_TRUE(fs::exists(dir / "frames/r1/2000_rgb.jpg"));
  const auto depth =
      cv::imread((dir / "frames/r1/1000_depth.png").string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(depth.type(), CV_16UC1);
  EXPECT_EQ(depth.at<uint16_t>(0, 0), 1500);

  rows = test::queryStore(
      dir,
      "SELECT obs_id, robot_id, track_id, frame_id, timestamp_ns, bbox2d_min_x, "
      "bbox2d_min_y, bbox2d_max_x, bbox2d_max_y, bbox3d_min_x, bbox3d_max_z, mask_rle, "
      "mask_path, score, kept FROM observations ORDER BY obs_id");
  ASSERT_EQ(rows.size(), 2u);
  const auto mask_path = "masks/r1/" + std::to_string(obs_ids[0]) + ".png";
  EXPECT_EQ(rows[0],
            (std::vector<std::string>{id(obs_ids[0]),
                                      "1",
                                      "12",
                                      id(obs_frame_id),
                                      "2000",
                                      "2",
                                      "1",
                                      "4",
                                      "2",
                                      "-1.0",
                                      "3.0",
                                      "[10,3,5,3,11]",
                                      mask_path,
                                      "2.5",
                                      "1"}));
  EXPECT_EQ(rows[1][12], "NULL");
  EXPECT_EQ(rows[1][14], "0");
  const auto mask = cv::imread((dir / mask_path).string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(mask.type(), CV_8UC1);
  EXPECT_EQ(cv::countNonZero(mask), 6);

  EXPECT_EQ(test::queryValue(dir, "SELECT node_id FROM track_bindings"), id(object));
  EXPECT_EQ(test::queryValue(dir, "SELECT parent_id FROM node_aliases"), id(object));
}

// Frames are unique per sensor and timestamp; later additions fill in what is missing
TEST(ImageStore, FrameDedup) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  EXPECT_EQ(store->addSensor(0, "camera", makeCalib()), sensor);
  const auto other_sensor = store->addSensor(0, "other", makeCalib());
  EXPECT_NE(sensor, other_sensor);

  // an observation frame without images, later found to be a keyframe
  const auto frame_id =
      store->addFrame(makeFrame(sensor, 1000, ImageStore::FrameKind::Observation));
  EXPECT_EQ(store->unanchoredFrames(0, 1000).size(), 1u);
  auto keyframe = makeFrame(sensor, 1000, ImageStore::FrameKind::Keyframe);
  keyframe.anchor_node = NodeSymbol('a', 1);
  EXPECT_EQ(store->addFrame(keyframe, makeColor()), frame_id);
  EXPECT_TRUE(store->unanchoredFrames(0, 1000).empty());
  EXPECT_EQ(store->addFrame(keyframe, makeColor(), makeDepth()), frame_id);

  // same timestamp from another sensor is another frame
  const auto other_id = store->addFrame(
      makeFrame(other_sensor, 1000, ImageStore::FrameKind::Keyframe), makeColor());
  EXPECT_NE(other_id, frame_id);
  store->flush();

  EXPECT_EQ(count(tmp.path, "frames"), "2");
  const auto rows = test::queryStore(tmp.path,
                                     "SELECT kind, anchor_node_id, rgb_path, "
                                     "depth_path FROM frames WHERE frame_id = " +
                                         id(frame_id));
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0],
            (std::vector<std::string>{"0",
                                      id(NodeSymbol('a', 1)),
                                      "frames/r0/1000_rgb.jpg",
                                      "frames/r0/1000_depth.png"}));
  EXPECT_EQ(
      test::queryValue(tmp.path,
                       "SELECT rgb_path FROM frames WHERE frame_id = " + id(other_id)),
      "frames/r0/1000_s1_rgb.jpg");
}

TEST(ImageStore, UnanchoredFrames) {
  test::TempDirectory tmp;
  ImageStore::Config config;
  config.path = tmp.path;
  config.write_depth = false;
  auto store = ImageStore::open(config);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  const auto robot_sensor = store->addSensor(1, "camera", makeCalib());
  std::vector<uint64_t> ids;
  for (const uint64_t stamp : {300u, 100u, 200u}) {
    ids.push_back(store->addFrame(
        makeFrame(sensor, stamp, ImageStore::FrameKind::SubKeyframe), makeColor()));
  }

  store->addFrame(makeFrame(robot_sensor, 100, ImageStore::FrameKind::SubKeyframe, 1));

  // in time order, up to the timestamp
  auto pending = store->unanchoredFrames(0, 250);
  ASSERT_EQ(pending.size(), 2u);
  EXPECT_EQ(pending[0].frame_id, ids[1]);
  EXPECT_EQ(pending[1].frame_id, ids[2]);
  EXPECT_TRUE(pending[0].world_T_body.translation().isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_EQ(store->unanchoredFrames(1, 1000).size(), 1u);

  Eigen::Isometry3d anchor_T_body = Eigen::Isometry3d::Identity();
  anchor_T_body.translation().x() = 0.5;
  store->setAnchor(ids[1], NodeSymbol('a', 2), anchor_T_body);
  store->abandonAnchor(ids[2]);
  pending = store->unanchoredFrames(0, 1000);
  ASSERT_EQ(pending.size(), 1u);
  EXPECT_EQ(pending[0].frame_id, ids[0]);
  store->flush();

  EXPECT_EQ(
      test::queryValue(
          tmp.path, "SELECT anchor_node_id FROM frames WHERE frame_id = " + id(ids[1])),
      id(NodeSymbol('a', 2)));
  EXPECT_EQ(
      test::queryValue(
          tmp.path, "SELECT anchor_node_id FROM frames WHERE frame_id = " + id(ids[2])),
      "NULL");
  EXPECT_EQ(test::queryValue(tmp.path, "SELECT COUNT(depth_path) FROM frames"), "0");
}

// Merges mirror the backend: entries are added, updated and removed
TEST(ImageStore, MergeAliases) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  const NodeId o1 = NodeSymbol('O', 1);
  const NodeId o2 = NodeSymbol('O', 2);
  const NodeId o3 = NodeSymbol('O', 3);
  const NodeId o4 = NodeSymbol('O', 4);
  store->recordMerges({{o2, o1}, {o3, o1}});
  store->recordMerges({{o2, o1}, {o3, o4}});  // merges can be recomputed
  store->recordMerges({}, {o2});
  store->flush();

  const auto rows = test::queryStore(
      tmp.path, "SELECT child_id, parent_id FROM node_aliases ORDER BY child_id");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0], (std::vector<std::string>{id(o3), id(o4)}));
}

// Bindings are upserts
TEST(ImageStore, TrackBindingsUpsert) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  store->bindTrack(0, 5, NodeSymbol('O', 1));
  store->bindTrack(0, 5, NodeSymbol('O', 1));
  store->bindTrack(0, 5, NodeSymbol('O', 2));
  store->bindTrack(1, 5, NodeSymbol('O', 3));
  store->flush();
  EXPECT_EQ(count(tmp.path, "track_bindings"), "2");
  EXPECT_EQ(test::queryValue(tmp.path,
                             "SELECT node_id FROM track_bindings WHERE robot_id = 0"),
            id(NodeSymbol('O', 2)));
}

// Unbinding removes the row; unknown tracks are ignored
TEST(ImageStore, UnbindTrack) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  store->bindTrack(0, 5, NodeSymbol('O', 1));
  store->bindTrack(0, 6, NodeSymbol('O', 2));
  store->unbindTrack(0, 5);
  store->unbindTrack(0, 9);
  store->flush();
  EXPECT_EQ(count(tmp.path, "track_bindings"), "1");
  store->bindTrack(0, 5, NodeSymbol('O', 1));
  store->flush();
  EXPECT_EQ(count(tmp.path, "track_bindings"), "2");
}

// Ids continue after the largest id of each robot when a store is reopened
TEST(ImageStore, ReopenResumesSequences) {
  test::TempDirectory tmp;
  uint64_t sensor;
  {
    auto store = ImageStore::open(tmp.path);
    ASSERT_TRUE(store);
    sensor = store->addSensor(2, "camera", makeCalib());
    store->addFrame(makeFrame(sensor, 100, ImageStore::FrameKind::Keyframe, 2));
    const auto frame =
        store->addFrame(makeFrame(sensor, 200, ImageStore::FrameKind::Keyframe, 2));
    EXPECT_EQ(frame, ImageStore::makeId(2, 1));
    ImageStore::Observation obs;
    obs.frame_id = frame;
    store->addObservations(2, 1, {obs, obs, obs});
  }

  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  EXPECT_EQ(store->addSensor(2, "camera", makeCalib()), sensor);
  EXPECT_EQ(store->addSensor(2, "other", makeCalib()), ImageStore::makeId(2, 1));
  // existing frames are still deduplicated
  EXPECT_EQ(store->addFrame(makeFrame(sensor, 200, ImageStore::FrameKind::Keyframe, 2)),
            ImageStore::makeId(2, 1));
  EXPECT_EQ(store->addFrame(makeFrame(sensor, 300, ImageStore::FrameKind::Keyframe, 2)),
            ImageStore::makeId(2, 2));
  // other robots start at zero
  const auto robot0_sensor = store->addSensor(0, "camera", makeCalib());
  EXPECT_EQ(
      store->addFrame(makeFrame(robot0_sensor, 300, ImageStore::FrameKind::Keyframe)),
      ImageStore::makeId(0, 0));
  ImageStore::Observation obs;
  obs.frame_id = ImageStore::makeId(2, 2);
  EXPECT_EQ(store->addObservations(2, 1, {obs}).front(), ImageStore::makeId(2, 3));
  store->flush();
  EXPECT_EQ(count(tmp.path, "frames"), "4");
  EXPECT_EQ(count(tmp.path, "observations"), "4");
}

// Ids taken by another writer of the same store are replaced when committed
TEST(ImageStore, OtherWriterIds) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  store->addFrame(makeFrame(sensor, 100, ImageStore::FrameKind::Keyframe));
  store->flush();

  // e.g., the python package adds frames with the next ids
  sqlite3* db = nullptr;
  ASSERT_EQ(sqlite3_open((tmp.path / "store.sqlite").c_str(), &db), SQLITE_OK);
  sqlite3_busy_timeout(db, 5000);
  const std::string insert =
      "INSERT INTO frames (frame_id, robot_id, sensor_id, timestamp_ns, kind, "
      "world_T_body_odom) VALUES ";
  ASSERT_EQ(sqlite3_exec(db,
                         (insert + "(1, 0, " + id(sensor) +
                          ", 300, 2, x'00'), (7, 0, " + id(sensor) + ", 500, 2, x'00')")
                             .c_str(),
                         nullptr,
                         nullptr,
                         nullptr),
            SQLITE_OK);
  sqlite3_close(db);

  const auto frame_400 = store->addFrame(
      makeFrame(sensor, 400, ImageStore::FrameKind::SubKeyframe), makeColor());
  EXPECT_EQ(frame_400, 1u);  // allocated before the other writer is noticed
  auto keyframe = makeFrame(sensor, 500, ImageStore::FrameKind::Keyframe);
  keyframe.anchor_node = NodeSymbol('a', 1);
  store->addFrame(keyframe, makeColor());
  ImageStore::Observation obs;
  obs.frame_id = frame_400;
  store->addObservations(0, 3, {obs});
  store->setAnchor(frame_400, NodeSymbol('a', 2), Eigen::Isometry3d::Identity());
  store->flush();

  EXPECT_EQ(count(tmp.path, "frames"), "4");
  // the frame that lost its id gets the next free id
  EXPECT_EQ(test::queryValue(tmp.path,
                             "SELECT frame_id || ':' || anchor_node_id FROM frames "
                             "WHERE timestamp_ns = 400"),
            "8:" + id(NodeSymbol('a', 2)));
  EXPECT_EQ(test::queryValue(tmp.path, "SELECT frame_id FROM observations"), "8");
  // the frame the other writer added is reused
  EXPECT_EQ(
      test::queryValue(tmp.path,
                       "SELECT frame_id || ':' || kind || ':' || anchor_node_id || "
                       "':' || rgb_path FROM frames WHERE timestamp_ns = 500"),
      "7:0:" + id(NodeSymbol('a', 1)) + ":frames/r0/500_rgb.jpg");
  // later ids continue after the largest id
  EXPECT_EQ(store->addFrame(makeFrame(sensor, 600, ImageStore::FrameKind::Keyframe)),
            9u);
}

// A failing operation leaves none of its rows behind and does not affect the others
TEST(ImageStore, FailedOperationIsRolledBack) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  const auto frame =
      store->addFrame(makeFrame(sensor, 100, ImageStore::FrameKind::Observation));
  store->flush();

  // e.g., a constraint of another writer
  sqlite3* db = nullptr;
  ASSERT_EQ(sqlite3_open((tmp.path / "store.sqlite").c_str(), &db), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(db,
                         "CREATE TRIGGER reject AFTER INSERT ON observations WHEN "
                         "NEW.score < 0 BEGIN SELECT RAISE(ABORT, 'rejected'); END",
                         nullptr,
                         nullptr,
                         nullptr),
            SQLITE_OK);
  sqlite3_close(db);

  ImageStore::Observation good;
  good.frame_id = frame;
  good.score = 1.0;
  auto bad = good;
  bad.score = -1.0;
  store->addObservations(0, 3, {good, bad});
  store->addObservations(0, 4, {good});
  store->bindTrack(0, 3, NodeSymbol('O', 3));
  store->flush();
  EXPECT_EQ(
      test::queryValue(tmp.path, "SELECT group_concat(track_id) FROM observations"),
      "4");
  EXPECT_EQ(count(tmp.path, "track_bindings"), "1");
}

// Masks are written before the transaction without replacing existing files, which
// are only replaced once the id is known to be free
TEST(ImageStore, MaskReplacesStaleFile) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  const auto frame =
      store->addFrame(makeFrame(sensor, 100, ImageStore::FrameKind::Observation));
  store->flush();

  // e.g., left by a write that failed before it was committed
  writeFile(tmp.path / "masks/r0/0.png", "other");
  ImageStore::Observation obs;
  obs.frame_id = frame;
  obs.kept = true;
  obs.mask = cv::Mat::zeros(4, 8, CV_8UC1);
  obs.mask(cv::Rect(0, 0, 2, 2)).setTo(255);
  const auto ids = store->addObservations(0, 3, {obs});
  ASSERT_EQ(ids.size(), 1u);
  store->flush();

  // the id is free in the database, so the stale file is replaced
  EXPECT_EQ(test::queryValue(tmp.path, "SELECT mask_path FROM observations"),
            "masks/r0/0.png");
  const auto mask =
      cv::imread((tmp.path / "masks/r0/0.png").string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(mask.type(), CV_8UC1);
  EXPECT_EQ(cv::countNonZero(mask), 4);
}

// Frames of a robot must use a sensor of the same robot
TEST(ImageStore, RejectsSensorOfOtherRobot) {
  test::TempDirectory tmp;
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(1, "camera", makeCalib());
  EXPECT_THROW(
      store->addFrame(makeFrame(sensor, 100, ImageStore::FrameKind::Keyframe, 2)),
      std::invalid_argument);
}

// Readers can query the database while it is written
TEST(ImageStore, ConcurrentReader) {
  test::TempDirectory tmp;
  ImageStore::Config config;
  config.path = tmp.path;
  config.flush_period_ms = 1;
  config.max_queue_size = 8;
  auto store = ImageStore::open(config);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  store->flush();

  std::atomic<bool> done{false};
  std::atomic<size_t> num_reads{0};
  std::atomic<bool> monotonic{true};
  std::thread reader([&] {
    int64_t last = 0;
    while (!done) {
      sqlite3* db = nullptr;
      if (sqlite3_open_v2((tmp.path / "store.sqlite").c_str(),
                          &db,
                          SQLITE_OPEN_READONLY,
                          nullptr) != SQLITE_OK) {
        sqlite3_close(db);
        continue;
      }

      sqlite3_busy_timeout(db, 1000);
      sqlite3_stmt* stmt = nullptr;
      if (sqlite3_prepare_v2(
              db, "SELECT COUNT(*), COUNT(rgb_path) FROM frames", -1, &stmt, nullptr) ==
              SQLITE_OK &&
          sqlite3_step(stmt) == SQLITE_ROW) {
        const auto value = sqlite3_column_int64(stmt, 0);
        monotonic = monotonic && value >= last;
        last = value;
        ++num_reads;
      }

      sqlite3_finalize(stmt);
      sqlite3_close(db);
    }
  });

  for (uint64_t i = 0; i < 200; ++i) {
    store->addFrame(makeFrame(sensor, 1000 + i, ImageStore::FrameKind::SubKeyframe),
                    makeColor());
  }

  store->flush();
  done = true;
  reader.join();
  EXPECT_TRUE(monotonic);
  EXPECT_GT(num_reads, 0u);
  EXPECT_EQ(count(tmp.path, "frames"), "200");
  EXPECT_EQ(test::queryValue(tmp.path, "SELECT COUNT(rgb_path) FROM frames"), "200");
}

// Queued work is committed when the last reference is dropped
TEST(ImageStore, DrainsOnDestroy) {
  test::TempDirectory tmp;
  ImageStore::Config config;
  config.path = tmp.path;
  config.flush_period_ms = 60000;
  {
    auto store = ImageStore::open(config);
    ASSERT_TRUE(store);
    const auto sensor = store->addSensor(0, "camera", makeCalib());
    for (uint64_t i = 0; i < 20; ++i) {
      store->addFrame(makeFrame(sensor, i, ImageStore::FrameKind::SubKeyframe),
                      makeColor(),
                      makeDepth());
    }
  }

  EXPECT_EQ(count(tmp.path, "frames"), "20");
  EXPECT_TRUE(fs::exists(tmp.path / "frames/r0/19_rgb.jpg"));
  EXPECT_TRUE(fs::exists(tmp.path / "frames/r0/19_depth.png"));
}

// Stores written with another schema version are not opened
TEST(ImageStore, RejectsOtherSchemaVersion) {
  test::TempDirectory tmp;
  ASSERT_TRUE(ImageStore::open(tmp.path));  // created and closed again
  sqlite3* db = nullptr;
  ASSERT_EQ(sqlite3_open((tmp.path / "store.sqlite").c_str(), &db), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(db,
                         "UPDATE meta SET value = '99' WHERE key = 'schema_version'",
                         nullptr,
                         nullptr,
                         nullptr),
            SQLITE_OK);
  sqlite3_close(db);
  EXPECT_FALSE(ImageStore::open(tmp.path));
}

// A store left by an earlier session is moved to sessions/<k> with its scene graph
TEST(ImageStore, RotatesEarlierSession) {
  test::TempDirectory tmp;
  const auto run = tmp.path / "run";
  const auto path = run / "image_store";
  makeEarlierSession(path, 3);
  writeFile(run / "hydra/backend/dsg.json", "backend graph");
  writeFile(run / "hydra/frontend/dsg.json", "frontend graph");
  writeFile(run / "hydra/frontend/mesh.ply", "mesh");
  writeFile(run / "hydra/timing/ignored.csv", "timing");

  ImageStore::Config config;
  config.path = path;
  config.on_existing = ImageStore::OnExisting::Rotate;
  config.session_artifacts_dir = run / "hydra";
  auto store = ImageStore::open(config);
  ASSERT_TRUE(store);

  // the new session starts empty and numbers its ids from zero again
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  EXPECT_EQ(store->addFrame(makeFrame(sensor, 100, ImageStore::FrameKind::Keyframe)),
            ImageStore::makeId(0, 0));
  store->flush();
  EXPECT_EQ(count(path, "frames"), "1");
  EXPECT_EQ(count(path, "track_bindings"), "0");
  EXPECT_EQ(count(path, "node_aliases"), "0");
  EXPECT_FALSE(fs::exists(path / "frames/r0/100_rgb.jpg"));

  // the earlier session is intact under sessions/1
  const auto session = run / "sessions/1";
  const auto archived = session / "image_store";
  EXPECT_EQ(count(archived, "frames"), "3");
  EXPECT_EQ(count(archived, "track_bindings"), "1");
  EXPECT_EQ(count(archived, "node_aliases"), "1");
  EXPECT_TRUE(fs::exists(archived / "frames/r0/300_rgb.jpg"));
  EXPECT_EQ(test::queryValue(archived,
                             "SELECT rgb_path FROM frames WHERE timestamp_ns = 300"),
            "frames/r0/300_rgb.jpg");

  // with a copy of its scene graph (the originals stay where they are)
  EXPECT_EQ(readFile(session / "hydra/backend/dsg.json"), "backend graph");
  EXPECT_EQ(readFile(session / "hydra/frontend/dsg.json"), "frontend graph");
  EXPECT_EQ(readFile(session / "hydra/frontend/mesh.ply"), "mesh");
  EXPECT_FALSE(fs::exists(session / "hydra/timing"));
  EXPECT_EQ(readFile(run / "hydra/backend/dsg.json"), "backend graph");

  const auto summary = readFile(session / "session.json");
  EXPECT_NE(summary.find("\"session\": \"1\""), std::string::npos) << summary;
  EXPECT_NE(summary.find("\"store\": \"image_store\""), std::string::npos) << summary;
  EXPECT_NE(summary.find("\"keyframes\": 3"), std::string::npos) << summary;
  EXPECT_NE(summary.find("\"track_bindings\": 1"), std::string::npos) << summary;
  EXPECT_NE(summary.find("\"robot_ids\": [0]"), std::string::npos) << summary;
  EXPECT_NE(summary.find("\"artifacts_dir\": \"hydra\""), std::string::npos) << summary;
  EXPECT_NE(summary.find("\"backend/dsg.json\""), std::string::npos) << summary;
  EXPECT_NE(summary.find("\"artifacts_missing\": [\"backend/dsg_with_mesh.json\""),
            std::string::npos)
      << summary;
  EXPECT_EQ(numEntries(run / "sessions"), 1u);
}

// Empty or missing store directories are opened in place
TEST(ImageStore, EmptyStoreDoesNotRotate) {
  test::TempDirectory tmp;
  fs::create_directories(tmp.path / "empty/image_store");
  EXPECT_TRUE(openRotating(tmp.path / "empty/image_store"));
  EXPECT_TRUE(openRotating(tmp.path / "missing/image_store"));
  EXPECT_FALSE(fs::exists(tmp.path / "empty/sessions"));
  EXPECT_FALSE(fs::exists(tmp.path / "missing/sessions"));
  EXPECT_TRUE(fs::exists(tmp.path / "missing/image_store/store.sqlite"));
}

// Sessions are numbered after the largest existing session
TEST(ImageStore, SessionIndexIncrements) {
  test::TempDirectory tmp;
  const auto run = tmp.path / "run";
  fs::create_directories(run / "sessions/1");
  fs::create_directories(run / "sessions/3");
  fs::create_directories(run / "sessions/notes");
  makeEarlierSession(run / "image_store", 2);
  ASSERT_TRUE(openRotating(run / "image_store"));
  EXPECT_EQ(count(run / "sessions/4/image_store", "frames"), "2");

  makeEarlierSession(run / "other_store", 1);
  ASSERT_TRUE(openRotating(run / "other_store"));
  EXPECT_EQ(count(run / "sessions/5/other_store", "frames"), "1");
  EXPECT_FALSE(fs::exists(run / "sessions/2"));
  // no artifacts configured
  const auto summary = readFile(run / "sessions/5/session.json");
  EXPECT_NE(summary.find("\"artifacts_dir\": null"), std::string::npos) << summary;
}

// on_existing: append (the default) keeps writing into the earlier session's store
TEST(ImageStore, AppendKeepsEarlierSession) {
  test::TempDirectory tmp;
  const auto path = tmp.path / "run/image_store";
  makeEarlierSession(path, 3);
  auto store = ImageStore::open(path);
  ASSERT_TRUE(store);
  const auto sensor = store->addSensor(0, "camera", makeCalib());
  EXPECT_EQ(store->addFrame(makeFrame(sensor, 400, ImageStore::FrameKind::Keyframe)),
            ImageStore::makeId(0, 3));
  store->flush();
  EXPECT_EQ(count(path, "frames"), "4");
  EXPECT_EQ(count(path, "track_bindings"), "1");
  EXPECT_FALSE(fs::exists(tmp.path / "run/sessions"));
}

// on_existing: fail refuses to open the earlier session's store
TEST(ImageStore, FailRefusesEarlierSession) {
  test::TempDirectory tmp;
  const auto path = tmp.path / "run/image_store";
  makeEarlierSession(path, 3);
  ImageStore::Config config;
  config.path = path;
  config.on_existing = ImageStore::OnExisting::Fail;
  EXPECT_FALSE(ImageStore::open(config));
  EXPECT_FALSE(ImageStore::open(config));
  EXPECT_EQ(count(path, "frames"), "3");
  EXPECT_FALSE(fs::exists(tmp.path / "run/sessions"));

  // an empty store opens normally
  config.path = tmp.path / "fresh/image_store";
  EXPECT_TRUE(ImageStore::open(config));
}

// Modules sharing the instance (and later opens in the process) rotate only once
TEST(ImageStore, SharedInstanceRotatesOnce) {
  test::TempDirectory tmp;
  const auto run = tmp.path / "run";
  makeEarlierSession(run / "image_store", 2);
  {
    const auto khronos = openRotating(run / "image_store");
    const auto hydra = openRotating(run / "image_store/");
    ASSERT_TRUE(khronos);
    EXPECT_EQ(khronos.get(), hydra.get());
    const auto sensor = hydra->addSensor(0, "camera", makeCalib());
    hydra->addFrame(makeFrame(sensor, 100, ImageStore::FrameKind::Keyframe));
  }

  // every instance was released: reopening finds this session's store
  const auto store = openRotating(run / "image_store");
  ASSERT_TRUE(store);
  EXPECT_EQ(numEntries(run / "sessions"), 1u);
  EXPECT_EQ(count(run / "image_store", "frames"), "1");
  EXPECT_EQ(count(run / "sessions/1/image_store", "frames"), "2");
}

}  // namespace hydra
