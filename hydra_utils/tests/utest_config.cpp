#include <hydra_utils/config.h>

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
  TestEnum type;
};

template <typename Visitor>
void visit_config(Visitor& v, FakeConfig& config) {
  config_parser::visit_config(v["foo"], config.foo);
  config_parser::visit_config(v["bar"], config.bar);
  config_parser::visit_config(v["a"], config.a);
  config_parser::visit_config(v["b"], config.b);
  config_parser::visit_config(v["c"], config.c);
  config_parser::visit_config(v["msg"], config.msg);
  config_parser::visit_config(v["values"], config.values);
  config_parser::visit_config(v["type"], config.type);
}

DECLARE_CONFIG_OSTREAM_OPERATOR(FakeConfig)

struct FakeConfig2 {
  FakeConfig fake_config;
  std::string msg = "world";
};

template <typename Visitor>
void visit_config(Visitor& v, FakeConfig2& config) {
  config_parser::visit_config(v["fake_config"], config.fake_config);
  config_parser::visit_config(v["msg"], config.msg);
}

std::ostream& operator<<(std::ostream& out, TestEnum v) {
  switch (v) {
    case TestEnum::RED:
      out << "RED";
      return out;
    case TestEnum::GREEN:
      out << "GREEN";
      return out;
    case TestEnum::BLUE:
      out << "BLUE";
      return out;
    default:
      out << "INVALID";
      return out;
  }
}

TestEnum readFromString(const std::string& enum_string) {
  auto to_compare = config_parser::to_uppercase(enum_string);

  if (to_compare == "RED") {
    return TestEnum::RED;
  }
  if (to_compare == "GREEN") {
    return TestEnum::GREEN;
  }
  if (to_compare == "BLUE") {
    return TestEnum::BLUE;
  }

  return TestEnum::RED;
}

void readRosParam(const ros::NodeHandle& nh, const std::string& name, TestEnum& value) {
  std::string color = "";
  if (!nh.getParam(name, color)) {
    return;
  }

  value = readFromString(color);
}

}  // namespace hydra_utils

namespace YAML {
template <>
struct convert<hydra_utils::TestEnum> {
  static Node encode(const hydra_utils::TestEnum& rhs) {
    std::stringstream ss;
    ss << rhs;
    return Node(ss.str());
  }

  static bool decode(const Node& node, hydra_utils::TestEnum& rhs) {
    if (node.IsNull()) {
      return false;
    }
    rhs = hydra_utils::readFromString(node.as<std::string>());
    return true;
  }
};

}  // namespace YAML

// make sure the ros side compiles (even if we're not directly testing)
hydra_utils::FakeConfig load_from_ros() {
  return config_parser::load_from_ros<hydra_utils::FakeConfig>("~");
}

template <>
struct config_parser::is_config<hydra_utils::FakeConfig> : std::true_type {};

template <>
struct config_parser::is_config<hydra_utils::FakeConfig2> : std::true_type {};

// make sure our ostream operator works (even if we're not directly testing)
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

  std::vector<int> expected_values{1, 2, 3};
  EXPECT_EQ(config.values, expected_values);
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

  std::vector<int> expected_values{1, 2, 3};
  EXPECT_EQ(config.fake_config.values, expected_values);
  EXPECT_EQ(config.msg, "again");
}
