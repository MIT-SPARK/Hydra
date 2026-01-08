#include <gtest/gtest.h>
#include <hydra/utils/misc.h>

#include <numeric>

namespace hydra::utils {

struct RemoveConfiguration {
  std::string name;
  std::vector<size_t> to_remove;
  std::vector<size_t> expected;
};

struct MiscRemoveFixture : public testing::TestWithParam<RemoveConfiguration> {};

RemoveConfiguration test_configurations[] = {
    {"none", {}, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
    {"all", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, {}},
    {"odds", {1, 3, 5, 7, 9}, {1, 3, 5, 7, 9}},
    {"evens", {0, 2, 4, 6, 8}, {2, 4, 6, 8, 10}},
    {"outer_thirds", {3, 4, 5}, {1, 2, 3, 7, 8, 9, 10}},
    {"middle_third", {0, 1, 2, 7, 8, 9}, {4, 5, 6, 7}},
};

INSTANTIATE_TEST_SUITE_P(Misc,
                         MiscRemoveFixture,
                         testing::ValuesIn(test_configurations),
                         [](const auto& info) { return info.param.name; });

TEST_P(MiscRemoveFixture, RemoveByIndexCorrect) {
  const auto config = GetParam();
  std::vector<size_t> result(10);
  std::iota(result.begin(), result.end(), 1);
  remove_by_index(result, config.to_remove);
  EXPECT_EQ(result, config.expected);
}

}  // namespace hydra::utils
