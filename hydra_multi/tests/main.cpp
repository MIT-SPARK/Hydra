#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <kimera_pgmo/utils/logging.h>

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = 1;

  logging::Logger::addSink("cout", std::make_shared<logging::CoutSink>());
  ::testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
