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
#include <hydra_utils/config.h>
#include <hydra_utils/eigen_config_types.h>

#include <ros/package.h>

#include <gtest/gtest.h>

std::string get_test_path() {
  return ros::package::getPath("hydra_utils") + "/tests/resources/";
}

namespace hydra {

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

struct BarConfig {
  int a = 1;
  float b = 2.0f;
  std::string c = "test";
};

struct BarMapConfig {
  std::map<std::string, BarConfig> configs;
};

struct MapConverter {
  MapConverter() = default;

  std::map<TestEnum, bool> to(const std::map<std::string, bool>& other) const;

  std::map<std::string, bool> from(const std::map<TestEnum, bool>& other) const;
};

template <typename Visitor>
void visit_config(const Visitor& v, FakeConfig& config) {
  v.visit("foo", config.foo);
  v.visit("bar", config.bar);
  v.visit("a", config.a);
  v.visit("b", config.b);
  v.visit("c", config.c);
  v.visit("msg", config.msg);
  v.visit("values", config.values);
  v.visit("value_map", config.value_map);
  v.visit("type", config.type);
  v.visit("vec", config.vec);
  v.visit("enable_map", config.enable_map, MapConverter());
}

template <typename Visitor>
void visit_config(const Visitor& v, FakeConfig2& config) {
  v.visit("fake_config", config.fake_config);
  v.visit("msg", config.msg);
}

template <typename Visitor>
void visit_config(const Visitor& v, BarConfig& config) {
  v.visit("a", config.a);
  v.visit("b", config.b);
  v.visit("c", config.c);
}

template <typename Visitor>
void visit_config(const Visitor& v, BarMapConfig& config) {
  v.visit("configs", config.configs);
}

struct TestLogger : config_parser::Logger {
  TestLogger() = default;

  ~TestLogger() = default;

  inline void log_missing(const std::string& message) const override {
    ss << message << std::endl;
  }

  mutable std::stringstream ss;
};

}  // namespace hydra

namespace config_parser {

template <>
struct ConfigVisitor<std::map<std::string, hydra::BarConfig>> {
  using MapType = std::map<std::string, hydra::BarConfig>;

  template <typename V, typename std::enable_if<is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& v, MapType& value) {
    for (const auto& child : v.children()) {
      value[child] = hydra::BarConfig();
      v.visit(child, value[child]);
    }
  }

  template <typename V,
            typename std::enable_if<!is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& v, MapType& value) {
    v.pre_visit();
    v.post_visit();
    for (auto& kv_pair : value) {
      v.visit(kv_pair.first, kv_pair.second);
    }
  }
};

}  // namespace config_parser

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, FakeConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, FakeConfig2)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, BarConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, BarMapConfig)

DECLARE_CONFIG_ENUM(hydra,
                    TestEnum,
                    {TestEnum::RED, "RED"},
                    {TestEnum::GREEN, "GREEN"},
                    {TestEnum::BLUE, "BLUE"})

namespace hydra {

std::map<TestEnum, bool> MapConverter::to(
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

TEST(ConfigParser, RosName) {
  config_parser::RosParserImpl parser(ros::NodeHandle("/plain_test_config"));
  EXPECT_EQ(parser.name(), "/plain_test_config");

  auto new_parser = parser.child("foo");
  EXPECT_EQ(new_parser.name(), "/plain_test_config/foo");
}

TEST(ConfigParsing, RosChildren) {
  config_parser::RosParserImpl parser(ros::NodeHandle("/plain_test_config"));
  auto children = parser.children();
  EXPECT_EQ(children.size(), 11u);

  auto value_parser = parser.child("value_map");
  auto value_children = value_parser.children();
  EXPECT_EQ(value_children.size(), 2u);

  auto enable_parser = parser.child("enable_map");
  auto enable_children = enable_parser.children();
  EXPECT_EQ(enable_children.size(), 2u);
}

TEST(ConfigParsing, YamlChildren) {
  const std::string filepath = get_test_path() + "test_config.yaml";
  config_parser::YamlParserImpl parser(filepath);
  auto children = parser.children();
  EXPECT_EQ(children.size(), 11u);

  auto value_parser = parser.child("value_map");
  auto value_children = value_parser.children();
  EXPECT_EQ(value_children.size(), 2u);

  auto enable_parser = parser.child("enable_map");
  auto enable_children = enable_parser.children();
  EXPECT_EQ(enable_children.size(), 2u);
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

TEST(ConfigParsing, ParseMapStructYaml) {
  const std::string filepath = get_test_path() + "map_config.yaml";
  auto config = config_parser::load_from_yaml<BarMapConfig>(filepath);

  EXPECT_EQ(config.configs.size(), 2u);
  ASSERT_TRUE(config.configs.count("config_a"));
  ASSERT_TRUE(config.configs.count("config_b"));

  BarConfig config_a = config.configs["config_a"];
  EXPECT_EQ(config_a.a, -3);
  EXPECT_NEAR(config_a.b, 5.2f, 1.0e-6f);
  EXPECT_EQ(config_a.c, "hello");

  BarConfig config_b = config.configs["config_b"];
  EXPECT_EQ(config_b.a, 5);
  EXPECT_NEAR(config_b.b, -1.2f, 1.0e-6f);
  EXPECT_EQ(config_b.c, "other");
}

TEST(ConfigParsing, ParseMapStructRos) {
  auto config = config_parser::load_from_ros<BarMapConfig>("/map_test_config");

  EXPECT_EQ(config.configs.size(), 2u);
  ASSERT_TRUE(config.configs.count("config_a"));
  ASSERT_TRUE(config.configs.count("config_b"));

  BarConfig config_a = config.configs["config_a"];
  EXPECT_EQ(config_a.a, -3);
  EXPECT_NEAR(config_a.b, 5.2f, 1.0e-6f);
  EXPECT_EQ(config_a.c, "hello");

  BarConfig config_b = config.configs["config_b"];
  EXPECT_EQ(config_b.a, 5);
  EXPECT_NEAR(config_b.b, -1.2f, 1.0e-6f);
  EXPECT_EQ(config_b.c, "other");
}

TEST(ConfigParsing, OutputMapConfig) {
  const std::string filepath = get_test_path() + "map_config.yaml";
  auto config = config_parser::load_from_yaml<BarMapConfig>(filepath);

  std::stringstream ss;
  // endl to make expected easier to write
  ss << std::endl << config;

  std::string expected = R"out(
- configs:
  - config_a:
    - a: -3
    - b: 5.2
    - c: hello
  - config_b:
    - a: 5
    - b: -1.2
    - c: other
)out";

  EXPECT_EQ(expected, ss.str()) << "config:" << std::endl << ss.str();
}

TEST(ConfigParsing, TestMissing) {
  auto logger = std::make_shared<TestLogger>();
  const std::string filepath = get_test_path() + "missing_config.yaml";
  auto config = config_parser::load_from_yaml<BarConfig>(filepath, logger);

  std::string expected = R"out(
missing param /c. defaulting to test
)out";

  // newline makes it easier to write string
  std::stringstream result;
  result << std::endl << logger->ss.str();

  EXPECT_EQ(expected, result.str());
}

}  // namespace hydra
