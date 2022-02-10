#include <hydra_utils/config_parser.h>
#include <hydra_utils/ros_config.h>

#include <gtest/gtest.h>

class FakeParser {
 public:
  FakeParser() = default;

  explicit FakeParser(const std::string& name) : name_(name) {}

  FakeParser operator[](const std::string& name) const { return FakeParser(name); }

  template <typename T>
  void visit(T& val) const {
    std::cout << name_ << ": " << val << std::endl;
  }

 private:
  std::string name_;
};

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

hydra_utils::FakeConfig foo() {
  std::cout << std::endl << "*** FOO ***" << std::endl << std::endl;
  FakeParser parser;
  hydra_utils::FakeConfig config;
  config_parser::visit_config(parser, config);
  return config;
}

hydra_utils::FakeConfig2 bar() {
  std::cout << std::endl << "*** BAR ***" << std::endl << std::endl;
  FakeParser parser;
  hydra_utils::FakeConfig2 config;
  config_parser::visit_config(parser, config);
  return config;
}

hydra_utils::FakeConfig load_from_ros() {
  auto parser = config_parser::RosParser::Private();

  hydra_utils::FakeConfig config;
  config_parser::visit_config(parser, config);
  return config;
}

TEST(ConfigParsing, defaultTest) {
  foo();
  bar();
  SUCCEED();
}
