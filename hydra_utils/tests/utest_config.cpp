#include <hydra_utils/config.h>

#include <ros/package.h>

#include <gtest/gtest.h>

std::string get_test_path() {
  return ros::package::getPath("hydra_utils") + "/tests/resources/";
}

namespace hydra_utils {

struct FakeConfig {
  float foo = 5.0f;
  double bar = 10.0;
  int a = 1;
  uint8_t b = 2;
  int64_t c = -3;
  std::string msg = "hello";
};

template <typename Visitor>
void visit_config(Visitor& v, FakeConfig& config) {
  config_parser::visit_config(v["foo"], config.foo);
  config_parser::visit_config(v["bar"], config.bar);
  config_parser::visit_config(v["a"], config.a);
  config_parser::visit_config(v["b"], config.b);
  config_parser::visit_config(v["c"], config.c);
  config_parser::visit_config(v["msg"], config.msg);
}

struct FakeConfig2 {
  FakeConfig fake_config;
  std::string msg = "world";
};

template <typename Visitor>
void visit_config(Visitor& v, FakeConfig2& config) {
  config_parser::visit_config(v["fake_config"], config.fake_config);
  config_parser::visit_config(v["msg"], config.msg);
}

}  // namespace hydra_utils

hydra_utils::FakeConfig load_from_ros() {
  return config_parser::load_from_ros<hydra_utils::FakeConfig>("~");
}

template <>
struct config_parser::is_config<hydra_utils::FakeConfig> : std::true_type {};

template <>
struct config_parser::is_config<hydra_utils::FakeConfig2> : std::true_type {};

void show_config() {
  auto visitor = config_parser::ConfigDisplay(std::cout);

  hydra_utils::FakeConfig config;
  config_parser::visit_config(visitor, config);

  std::cout << config << std::endl;
}

TEST(ConfigParsing, ParseSingleStructYaml) {
  const std::string filepath = get_test_path() + "test_config.yaml";
  auto config = config_parser::load_from_yaml<hydra_utils::FakeConfig>(filepath);

  EXPECT_EQ(config.foo, 10.0f);
  EXPECT_EQ(config.bar, 5.0);
  EXPECT_EQ(config.a, -3);
  EXPECT_EQ(static_cast<int>(config.b), 1);
  EXPECT_EQ(config.c, 2);
  EXPECT_EQ(config.msg, "world");
}

TEST(ConfigParsing, ParseNestedStructYaml) {
  const std::string filepath = get_test_path() + "nested_test_config.yaml";
  auto config = config_parser::load_from_yaml<hydra_utils::FakeConfig2>(filepath);

  EXPECT_EQ(config.fake_config.foo, 10.0f);
  EXPECT_EQ(config.fake_config.bar, 5.0);
  EXPECT_EQ(config.fake_config.a, -3);
  EXPECT_EQ(static_cast<int>(config.fake_config.b), 1);
  EXPECT_EQ(config.fake_config.c, 2);
  EXPECT_EQ(config.fake_config.msg, "hello");
  EXPECT_EQ(config.msg, "again");
}
