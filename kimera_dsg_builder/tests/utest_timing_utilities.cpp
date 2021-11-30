#include "kimera_dsg_builder/timing_utilities.h"

#include <gtest/gtest.h>

#include <thread>

namespace kimera {

struct TimingUtilityTests : public ::testing::Test {
  virtual void SetUp() override { ElapsedTimeRecorder::instance().reset(); }
  virtual void TearDown() override { ElapsedTimeRecorder::instance().reset(); }
};

TEST_F(TimingUtilityTests, TestNoMeasurements) {
  EXPECT_FALSE(ElapsedTimeRecorder::instance().getLastElapsed("test"));

  auto stats = ElapsedTimeRecorder::instance().getStats("test");
  EXPECT_EQ(0u, stats.num_measurements);
}

TEST_F(TimingUtilityTests, TestSingleMeasurement) {
  ElapsedTimeRecorder::instance().start("test", 0);
  EXPECT_FALSE(ElapsedTimeRecorder::instance().getLastElapsed("test"));
  ElapsedTimeRecorder::instance().stop("test");

  auto elapsed = ElapsedTimeRecorder::instance().getLastElapsed("test");
  ASSERT_TRUE(elapsed);
  EXPECT_LT(0.0, *elapsed);

  auto stats = ElapsedTimeRecorder::instance().getStats("test");
  EXPECT_EQ(1u, stats.num_measurements);
  EXPECT_EQ(*elapsed, stats.last_s);
  EXPECT_EQ(*elapsed, stats.mean_s);
  EXPECT_EQ(0.0, stats.stddev_s);
}

TEST_F(TimingUtilityTests, TestMultipleMeasurements) {
  using namespace std::chrono_literals;

  ElapsedTimeRecorder::instance().start("test", 0);
  std::this_thread::sleep_for(10ms);
  ElapsedTimeRecorder::instance().stop("test");

  ElapsedTimeRecorder::instance().start("test", 0);
  std::this_thread::sleep_for(20ms);
  ElapsedTimeRecorder::instance().stop("test");

  ElapsedTimeRecorder::instance().start("test", 0);
  std::this_thread::sleep_for(30ms);
  ElapsedTimeRecorder::instance().stop("test");

  auto elapsed = ElapsedTimeRecorder::instance().getLastElapsed("test");
  ASSERT_TRUE(elapsed);
  EXPECT_NEAR(0.03, *elapsed, 1.0e-3);

  auto stats = ElapsedTimeRecorder::instance().getStats("test");
  EXPECT_EQ(3u, stats.num_measurements);
  EXPECT_EQ(*elapsed, stats.last_s);
  EXPECT_NEAR(0.02, stats.mean_s, 1.0e-2);
  EXPECT_NEAR(0.01, stats.stddev_s, 1.0e-2);
}

TEST_F(TimingUtilityTests, TestScopedTimers) {
  using namespace std::chrono_literals;

  ElapsedTimeRecorder::instance().start("test1", 0);
  ElapsedTimeRecorder::instance().start("test2", 0);
  {  // timer test3
    ScopedTimer timer3("test3", 0);

    {  // timer test 4
      ScopedTimer timer4("test4", 0);
      std::this_thread::sleep_for(2ms);
    }  // timer test 4

    std::this_thread::sleep_for(2ms);
    ElapsedTimeRecorder::instance().stop("test2");
    std::this_thread::sleep_for(4ms);

  }  // timer test2
  std::this_thread::sleep_for(2ms);
  ElapsedTimeRecorder::instance().stop("test1");

  auto elapsed_1 = ElapsedTimeRecorder::instance().getLastElapsed("test1");
  auto elapsed_2 = ElapsedTimeRecorder::instance().getLastElapsed("test2");
  auto elapsed_3 = ElapsedTimeRecorder::instance().getLastElapsed("test3");
  auto elapsed_4 = ElapsedTimeRecorder::instance().getLastElapsed("test4");

  ASSERT_TRUE(elapsed_1);
  ASSERT_TRUE(elapsed_2);
  ASSERT_TRUE(elapsed_3);
  ASSERT_TRUE(elapsed_4);

  // timer 1 ran the longest
  EXPECT_GT(*elapsed_1, *elapsed_2);
  EXPECT_GT(*elapsed_1, *elapsed_3);
  EXPECT_GT(*elapsed_1, *elapsed_4);

  // timer 3 ran the second longest
  EXPECT_GT(*elapsed_3, *elapsed_2);
  EXPECT_GT(*elapsed_3, *elapsed_4);

  // timer 4 ran the shortest
  EXPECT_GT(*elapsed_2, *elapsed_4);
}

}  // namespace kimera
