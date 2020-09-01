
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <boost/archive/tmpdir.hpp>

#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_serialization.h"

namespace kimera {

// Keep node handles persistent.
class SceneGraphSerializationTest : public testing::Test {
 public:
  SceneGraphSerialization() {
    filename_ = boost::archive::tmpdir() + "/testfile.txt";
  }
  ~SceneGraphSerialization() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

 protected:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;

  std::string filename_;
};

TEST_F(SceneGraphSerializationTest, SceneGraphEdgeSerialization) {
  // Default ctor
  SceneGraphEdge saved_edge;
  save(saved_edge, filename_.c_str());
  SceneGraphEdge loaded_edge;
  load(filename_.c_str(), &loaded_edge);
  EXPECT_TRUE(saved_edge.equal(loaded_edge));

  // Default ctor
  SceneGraphEdge saved_edge;
  saved_edge.edge_id_ = 10293;
  saved_edge.start_layer_id_ = LayerId::kAgentsLayerId;
  saved_edge.start_node_id_ = 1233;
  saved_edge.end_layer_id_ = LayerId::kObjectsLayerId;
  saved_edge.end_node_id_ = 3942;
  save(saved_edge, filename_.c_str());
  SceneGraphEdge loaded_edge;
  load(filename_.c_str(), &loaded_edge);
  EXPECT_TRUE(saved_edge.equal(loaded_edge));
}

}  // namespace kimera

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}
