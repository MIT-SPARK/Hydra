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
#include <gtest/gtest.h>
#include <hydra/utils/timing_utilities.h>

#include <thread>

namespace hydra {
namespace timing {

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

}  // namespace timing
}  // namespace hydra
