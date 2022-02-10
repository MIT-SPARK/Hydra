#include <hydra_utils/config_parser.h>
#include <hydra_utils/ros_config.h>

#include <gtest/gtest.h>

namespace kimera {

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
  config::visit_config(v["foo"], config.foo);
  config::visit_config(v["bar"], config.bar);
  config::visit_config(v["a"], config.a);
  config::visit_config(v["b"], config.b);
  config::visit_config(v["c"], config.c);
  config::visit_config(v["msg"], config.msg);
}

struct FakeConfig2 {
  FakeConfig fake_config;
  std::string msg = "world";
};

template <typename Visitor>
void visit_config(Visitor& v, FakeConfig2& config) {
  config::visit_config(v["fake_config"], config.fake_config);
  config::visit_config(v["msg"], config.msg);
}

FakeConfig foo() {
  std::cout << std::endl << "*** FOO ***" << std::endl << std::endl;
  FakeParser parser;
  FakeConfig config;
  config::visit_config(parser, config);
  return config;
}

FakeConfig2 bar() {
  std::cout << std::endl << "*** BAR ***" << std::endl << std::endl;
  FakeParser parser;
  FakeConfig2 config;
  config::visit_config(parser, config);
  return config;
}

TEST(ConfigParsing, defaultTest) {
  foo();
  bar();
  SUCCEED();
}

enum class FakeEnum { RED = 0, GREEN = 1, BLUE = 2 };

template <>
void showParam(std::ostream& out, const FakeEnum& value) {
  switch (value) {
    case FakeEnum::RED:
      out << "RED";
      break;
    case FakeEnum::GREEN:
      out << "GREEN";
      break;
    case FakeEnum::BLUE:
      out << "BLUE";
      break;
  }
  out << "(" << static_cast<int>(value) << ")";
}

bool parseParam(const ros::NodeHandle& nh, const std::string& name, FakeEnum& value) {
  std::string placeholder;
  bool had_param = nh.getParam(name, placeholder);
  if (!had_param) {
    return false;
  }

  std::transform(
      placeholder.begin(), placeholder.end(), placeholder.begin(), [](unsigned char c) {
        return std::toupper(c);
      });

  if (placeholder == "RED") {
    value = FakeEnum::RED;
    return true;
  }

  if (placeholder == "GREEN") {
    value = FakeEnum::GREEN;
    return true;
  }

  if (placeholder == "BLUE") {
    value = FakeEnum::BLUE;
    return true;
  }

  std::stringstream ss;
  ss << "Invalid FakeEnum: " << placeholder << ". Defaulting to ";
  showParam(ss, value);
  ROS_WARN_STREAM(ss.str());
  return true;
}

}  // namespace kimera
