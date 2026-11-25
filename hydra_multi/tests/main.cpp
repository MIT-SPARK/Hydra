#include <gtest/gtest.h>
#include <hydra/utils/logging.h>
#include <kimera_pgmo/utils/logging.h>

int main(int argc, char** argv) {
  hydra::logging::setLogLevel("warning");

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
