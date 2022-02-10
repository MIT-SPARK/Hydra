#include <hydra_utils/config_parser.h>
#include <hydra_utils/ros_config.h>

#include <gtest/gtest.h>

namespace kimera {

struct FakeConfig {
  float foo = 5.0f;
  double bar = 10.0;
  int a = 1;
  uint8_t b = 2;
  int64_t c = -3;
  std::string msg = "hello";
};

struct FakeParser {

  void visit(std::string& str) const {
    std::cout << str << std::endl;
  }

  void visit(float& val) const {
    std::cout << val << std::endl;
  }

};

template <typename Visitor>
void visit_config(Visitor& v, FakeConfig& config) {
  config::visit_config(v, config.foo);
  config::visit_config(v, config.msg);
}

FakeConfig foo() {
  // ros::NodeHandle nh;
  // RosParser parser(nh);

  FakeParser parser;

  FakeConfig config;
  config::visit_config(parser, config);
  return config;
}

TEST(ConfigParsing, defaultTest) {
  foo();
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
