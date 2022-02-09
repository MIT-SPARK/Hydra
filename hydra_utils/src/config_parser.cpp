#include "hydra_utils/config_parser.h"

namespace kimera {

template <typename T, typename C>
struct ConfigAccess {};

template <typename T>
struct ConfigAccess<T, FakeConfig> {
  static void call(const T& t, FakeConfig& config) {
    t.call("foo", config.foo);
    t.call("bar", config.bar);
    t.call("a", config.a);
    t.call("b", config.b);
    t.call("c", config.c);
    t.call("msg", config.msg);
  }
};

/*template <typename T>*/
/*struct FakeConfigAccessor {*/
/*using Config = FakeConfig;*/

/*static void call(const T& t, FakeConfig& config) {*/
/*t.call("foo", config.foo);*/
/*t.call("bar", config.bar);*/
/*t.call("a", config.a);*/
/*t.call("b", config.b);*/
/*t.call("c", config.c);*/
/*t.call("msg", config.msg);*/
/*}*/
/*};*/

struct RosParser {
  explicit RosParser(const ros::NodeHandle& nh) : nh_(nh) {}

  template <typename T>
  void call(const std::string& name, T& value) const {
    parseParam(nh_, name, value);
  }

  ros::NodeHandle nh_;
};

// template <template <typename> typename ConfigAccess, typename Parser>
// void load(const Parser& parser, typename ConfigAccess<Parser>::Config& config) {
template <typename Config, typename Parser>
void load(const Parser& parser, Config& config) {
  ConfigAccess<Parser>::call(parser, config);
}

FakeConfig foo() {
  ros::NodeHandle nh;
  RosParser parser(nh);

  FakeConfig config;
  load(parser, config);
  return config;
}

#define READ_PARAM(nh, config, name, bounds...)                 \
  if (!parseParam(nh, #name, config.name, ##bounds)) {          \
    std::stringstream ss;                                       \
    ss << nh.resolveName(#name) + " not found! Defaulting to "; \
    outputParamDefault(ss, config.name);                        \
    ROS_DEBUG_STREAM(ss.str());                                 \
  }                                                             \
  static_assert(true, "")

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
