#include <hydra_utils/config.h>
#include <hydra_utils/eigen_config_types.h>

#include <ros/package.h>

#include <gtest/gtest.h>

std::string get_test_path() {
  return ros::package::getPath("hydra_utils") + "/tests/resources/";
}

namespace hydra_utils {

enum class TestEnum { RED, GREEN, BLUE };

struct FakeConfig {
  float foo = 5.0f;
  double bar = 10.0;
  int a = 1;
  uint8_t b = 2;
  int64_t c = -3;
  std::string msg = "hello";
  std::vector<int> values{1, 2, 3};
  std::map<std::string, int> value_map{{"1", 2}, {"3", 4}};
  TestEnum type = TestEnum::RED;
  Eigen::Matrix<uint8_t, 3, 1> vec = Eigen::Matrix<uint8_t, 3, 1>::Zero();
  std::map<TestEnum, bool> enable_map{{TestEnum::RED, true}, {TestEnum::GREEN, false}};
};

struct FakeConfig2 {
  FakeConfig fake_config;
  std::string msg = "world";
};

struct MapConverter {
  MapConverter() = default;

  std::map<TestEnum, bool> from(const std::map<std::string, bool>& other) const;

  std::map<std::string, bool> from(const std::map<TestEnum, bool>& other) const;
};

template <typename Visitor>
void visit_config(const Visitor& v, FakeConfig& config) {
  config_parser::visit_config(v["foo"], config.foo);
  config_parser::visit_config(v["bar"], config.bar);
  config_parser::visit_config(v["a"], config.a);
  config_parser::visit_config(v["b"], config.b);
  config_parser::visit_config(v["c"], config.c);
  config_parser::visit_config(v["msg"], config.msg);
  config_parser::visit_config(v["values"], config.values);
  config_parser::visit_config(v["value_map"], config.value_map);
  config_parser::visit_config(v["type"], config.type);
  config_parser::visit_config(v["vec"], config.vec);
  config_parser::visit_config(v["enable_map"], config.enable_map, MapConverter());
}

template <typename Visitor>
void visit_config(const Visitor& v, FakeConfig2& config) {
  config_parser::visit_config(v["fake_config"], config.fake_config);
  config_parser::visit_config(v["msg"], config.msg);
}

}  // namespace hydra_utils

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra_utils, FakeConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra_utils, FakeConfig2)

DECLARE_CONFIG_ENUM(hydra_utils,
                    TestEnum,
                    {TestEnum::RED, "RED"},
                    {TestEnum::GREEN, "GREEN"},
                    {TestEnum::BLUE, "BLUE"})

namespace hydra_utils {

std::map<TestEnum, bool> MapConverter::from(
    const std::map<std::string, bool>& other) const {
  std::map<TestEnum, bool> to_return;
  for (const auto& kv_pair : other) {
    // defined by enum macro
    TestEnum new_enum = readTestEnumFromString(kv_pair.first);
    to_return[new_enum] = kv_pair.second;
  }
  return to_return;
}

std::map<std::string, bool> MapConverter::from(
    const std::map<TestEnum, bool>& other) const {
  std::map<std::string, bool> to_return;
  for (const auto& kv_pair : other) {
    std::stringstream ss;
    ss << kv_pair.first;
    to_return[ss.str()] = kv_pair.second;
  }

  return to_return;
}

TEST(ConfigParsing, ParseSingleStructYaml) {
  const std::string filepath = get_test_path() + "test_config.yaml";
  auto config = config_parser::load_from_yaml<FakeConfig>(filepath);

  EXPECT_EQ(config.foo, 10.0f);
  EXPECT_EQ(config.bar, 5.0);
  EXPECT_EQ(config.a, -3);
  EXPECT_EQ(static_cast<int>(config.b), 1);
  EXPECT_EQ(config.c, 2);
  EXPECT_EQ(config.msg, "world");

  std::vector<int> expected_values{4, 5, 6};
  EXPECT_EQ(config.values, expected_values);

  std::map<std::string, int> expected_value_map{{"3", 4}, {"5", 6}};
  EXPECT_EQ(config.value_map, expected_value_map);

  EXPECT_EQ(config.type, TestEnum::GREEN);

  Eigen::Matrix<uint8_t, 3, 1> expected_vec;
  expected_vec << 7, 8, 9;
  EXPECT_EQ(config.vec, expected_vec);

  std::map<TestEnum, bool> expected_enable_map{{TestEnum::BLUE, 0},
                                               {TestEnum::GREEN, 1}};
  EXPECT_EQ(config.enable_map, expected_enable_map);
}

TEST(ConfigParsing, ParseSingleStructRos) {
  auto config = config_parser::load_from_ros<FakeConfig>("/plain_test_config");

  EXPECT_EQ(config.foo, 10.0f);
  EXPECT_EQ(config.bar, 5.0);
  EXPECT_EQ(config.a, -3);
  EXPECT_EQ(static_cast<int>(config.b), 1);
  EXPECT_EQ(config.c, 2);
  EXPECT_EQ(config.msg, "world");

  std::vector<int> expected_values{4, 5, 6};
  EXPECT_EQ(config.values, expected_values);

  std::map<std::string, int> expected_value_map{{"3", 4}, {"5", 6}};
  EXPECT_EQ(config.value_map, expected_value_map);

  EXPECT_EQ(config.type, TestEnum::GREEN);

  Eigen::Matrix<uint8_t, 3, 1> expected_vec;
  expected_vec << 7, 8, 9;
  EXPECT_EQ(config.vec, expected_vec);

  std::map<TestEnum, bool> expected_enable_map{{TestEnum::BLUE, 0},
                                               {TestEnum::GREEN, 1}};
  EXPECT_EQ(config.enable_map, expected_enable_map);
}

TEST(ConfigParsing, ParseNestedStructYaml) {
  const std::string filepath = get_test_path() + "nested_test_config.yaml";
  auto config = config_parser::load_from_yaml<FakeConfig2>(filepath);

  EXPECT_EQ(config.fake_config.foo, 10.0f);
  EXPECT_EQ(config.fake_config.bar, 5.0);
  EXPECT_EQ(config.fake_config.a, -3);
  EXPECT_EQ(static_cast<int>(config.fake_config.b), 1);
  EXPECT_EQ(config.fake_config.c, 2);
  EXPECT_EQ(config.fake_config.msg, "hello");

  std::vector<int> expected_values{1, 2, 3};
  EXPECT_EQ(config.fake_config.values, expected_values);
  EXPECT_EQ(config.msg, "again");

  // make sure conversion respects the default
  std::map<TestEnum, bool> expected_enable_map{{TestEnum::RED, 1},
                                               {TestEnum::GREEN, 0}};
  EXPECT_EQ(config.fake_config.enable_map, expected_enable_map);
}

TEST(ConfigParsing, ParseNestedStructRos) {
  auto config = config_parser::load_from_ros<FakeConfig2>("/nested_test_config");
  ros::NodeHandle nh("/nested_test_config");

  EXPECT_EQ(config.fake_config.foo, 10.0f);
  EXPECT_EQ(config.fake_config.bar, 5.0);
  EXPECT_EQ(config.fake_config.a, -3);
  EXPECT_EQ(static_cast<int>(config.fake_config.b), 1);
  EXPECT_EQ(config.fake_config.c, 2);
  EXPECT_EQ(config.fake_config.msg, "hello");

  std::vector<int> expected_values{1, 2, 3};
  EXPECT_EQ(config.fake_config.values, expected_values);
  EXPECT_EQ(config.msg, "again");

  // make sure conversion respects the default
  std::map<TestEnum, bool> expected_enable_map{{TestEnum::RED, 1},
                                               {TestEnum::GREEN, 0}};
  EXPECT_EQ(config.fake_config.enable_map, expected_enable_map);
}

TEST(ConfigParsing, OutputSingleConfig) {
  FakeConfig config;
  std::stringstream ss;
  // endl to make expected easier to write
  ss << std::endl << config;

  std::string expected = R"out(
- foo: 5
- bar: 10
- a: 1
- b: 2
- c: -3
- msg: hello
- values: [1, 2, 3]
- value_map: {1: 2, 3: 4}
- type: RED
- vec: [0, 0, 0]
- enable_map: {GREEN: 0, RED: 1}
)out";

  EXPECT_EQ(expected, ss.str()) << "config:" << std::endl << ss.str();
}

TEST(ConfigParsing, OutputNestedConfig) {
  FakeConfig2 config;
  std::stringstream ss;
  // endl to make expected easier to write
  ss << std::endl << config;

  std::string expected = R"out(
- fake_config:
  - foo: 5
  - bar: 10
  - a: 1
  - b: 2
  - c: -3
  - msg: hello
  - values: [1, 2, 3]
  - value_map: {1: 2, 3: 4}
  - type: RED
  - vec: [0, 0, 0]
  - enable_map: {GREEN: 0, RED: 1}
- msg: world
)out";

  EXPECT_EQ(expected, ss.str()) << "config:" << std::endl << ss.str();
}

}  // namespace hydra_utils
