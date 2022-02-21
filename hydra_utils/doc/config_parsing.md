## Config Parsing

`hydra_utils` contains utilities to handle reading arbitrary structures from the
ROS parameter server or YAML files (or any configuration format theoretically).
This is accomplished primarily through several occurrences of argument-dependent
lookup (ADL) based specialization (and loosely modeled on the json library
[here](https://github.com/nlohmann/json)).


### Parsing and Output Utilities

Assuming a properly declared structure `FooConfig`, there several useful out-of-the-box features:

```cpp
#include <hydra_utils/config.h>
#include <iostream>

// load config from a global namespace
FooConfig config = config_parser::load_from_ros<FooConfig>("/node_ns");

// load config directly from the node's private namespace
config =  config_parser::load_from_ros_nh<FooConfig>(ros::NodeHandle("~"));

// load_from_ros_nh also supports an additional namespace argument

// load config directly from file
config = config_parser::load_from_yaml("/path/to/config.yaml");

// declared configs also have output operators for any ostream.
std::cout << config << std::endl;

```

See the main header file [here](../include/hydra_utils/config.h) for more details.

### Declaring a new Configuration Structure (or Class)

Assume that we have the following structure `FooConfig` that we want to declare so that we can enable the utilities in the previous section:

```cpp
namespace foo_ns {

// simple struct used in FooConfig
struct BarConfig {
    int a = 5;
    bool b = false;
};

struct FooConfig {
    std::string param_1 = "hello";            // Any base ros XmlRpcValue types are supported automatically
    int64_t param_2 = 50;                     // Arbitrary integral types are also supported
    std::vector<uint64_t> param_3 = {};       // As well as vectors of arbitrary integral types
    std::map<std::string, bool> param_4 = {}; // Maps with string keys and base ros types are supported
    BarConfig nested_config;                  // Nested structure are also supported
};

} // namespace foo_ns
```

We would declared our `FooConfig` structure like this:

```cpp
#include <hydra_utils/config.h>

namespace foo_ns {

template <typename Visitor>
void visit_config(const Visitor& v, FooConfig& config) {
    // typically param namespaces match the name of the field
    v.visit("param_1", config.param_1);
    // but they don't have too
    v.visit("bar", config.param_2);
    // we can also make a new "visitor" in a child namespace:
    auto nested_visitor = v["new_ns"];
    // we then can parse param_3 at "/new_ns/param_3"
    nested_visitor.visit("param_3", config.param_3);
    // this has no bearing on following parameters
    v.visit("param_4", config.param_4);

    // we can also recursively visit nested configurations (in this case, at the namespace "/bar")
    v.visit("bar", nested_config);
}

// enables parsing of BarConfig
template <typename Visitor>
void visit_config(const Visitor& v, BarConfig& config) {
    v.visit("a", config.a);
    v.visit("b", config.b);
}

} // namespace foo_ns

// Declare output operator for BarConfig
DECLARE_CONFIG_OSTREAM_OPERATOR(foo_ns, BarConfig)

// To make sure that our output operator can deduce when it is "entering" a new
// namespace corresponding to a nested config (e.g. nested_config in FooConfig),
// we need to declare some information before the output operator is actually
// defined, and we have to declare it in the config_parser namespace
// (specifically, a specialization of the config_parser::is_config template).
// Therefore, this macro needs to be called from the global namespace (so that
// we can declare the specialization of the is_config trait and also declare the
// operator inside FooConfig's namespace (so that it gets found in any namespace)
DECLARE_CONFIG_OSTREAM_OPERATOR(foo_ns, FooConfig)
```

:warning: `visit_config` must declare the config argument as non-const. This is
a side-effect of the parsing and output code both making use of `visit_config`
(parsing requires that the config is non-const so we can modify configuration fields).

This is all you need to do as long as the configuration structure is default constructible and only contains:

  * ROS primitive types (`bool`, `float`, `double`, `std::string`, `int`)
  * [Integral types](https://en.cppreference.com/w/cpp/types/is_integral), e.g. `uint8_t`, `int64_t`
  * Any `std::vector` of ROS primitive types or integral types
  * Any `std::map` with std::string for keys and ROS primitive types for values
  * Any structure that contains only members of the above (or other nested structures)

Note that this doesn't apply to any parts of the config you **do not** want to parse, as well as any class methods. This does mean that if you had some complex member type that doesn't follow these rules, you could conceivably do something like:

```cpp
template <typename Visitor>
void visit_config(const Visitor& v, SomeConfig& config) {
    // visit all of SomeConfig's members that are valid

    // you could also write a non-parser and parser version using SFINAE
    if (config_parser::is_parser<Visitor>()) {
        config.bar = SomeRandomType(config.a, config.b, ...);
    }
}
```

It can be tedious to maintain members used for the constructor, so you could also declare a version of `visit_config` for the member type.  For more primitive member types that aren't enabled by default, you can enable them by:

  * Defining a version of `readRosParam` (see [here](../include/hydra_utils/ros_parser.h) for details) in the **same** namespace as the type
  * Defining a version of `displayParam` (see [here](../include/hydra_utils/ostream_formatter.h) for details) in the **same** namespace as the type
  * Defining a specialization of `YAML::converter` in the `YAML` namespace

This can be tedious for enums, so this can be directly handled by a macro in `config.h`:

```cpp
namespace some_ns {

enum class FakeEnum { RED = 0, GREEN = 1, BLUE = 2 };

} // namespace some_ns

// must be in the global namespace so that we can specialize YAML::converter
DECLARE_CONFIG_ENUM(some_ns,
                    FakeEnum,
                    {FakeEnum::RED, "RED"},
                    {FakeEnum::GREEN, "GREEN"},
                    {FakeEnum::BLUE, "BLUE"})
```

### Maps, Collections, and Conversions

Occasionally, you might have to parse a struct like this:

```cpp
namespace fake_ns {

struct SomeStruct {
    std::map<int, SomeConfig> configs;
};

} // namespace fake_ns
```

There are two problems here. First, you might not specify integers as configuration keys or values (e.g. if the keys are enums and have nice human-readable strings), and you might want to parse a different type (e.g. `std::map<std::string, SomeConfig>`). `visit` is overloaded to take a `Converter`, which has a signature of:

```cpp
struct SomeConverter {
SomeConverter() = default;

ParsingType from(const OrigType& value) const {
    // convert from the configuration value to something the parsing understands
}

OrigType to(const ParsingType& value) const {
    // do the opposite conversion
}
};
```

and gets used like so:

```cpp
template <typename Visitor>
void visit_config(const Visitor& v, SomeStruct& config) {
    v.visit("configs", config.configs, SomeConverter());
}
```

However, this doesn't really work when you don't have an intermediate type to convert to for parsing purposes.  In this case, you have to specialize `visit_config` for the collection type in question. However, doing this for stl members requires working in the `std` namespace (a bad idea). Instead, you can specialize the `ConfigVisitor` struct like so:

```cpp
namespace config_parser {

template <>
ConfigVisitor<std::map<int, fake_ns::SomeConfig>> {

    // most likely you'd want to use SFINAE / enable_if to have a non-parsing version
    template <typename Visitor>
    static auto visit_config(const Visitor& v, std::map<int, fake_ns::SomeConfig>& value) {
        for (const std::string& child_name : v.children()) {
            int new_key = std::atoi(child_name);
            value[new_key] = fake_ns::SomeConfig();
            v.visit(child_name, value[new_key]);
        }
    }

};

} // namespace config_parser
```

### New Parsers or Formatters

Most of the recursive logic happens in `config_parser::Parser` or
`config_parser::Formatter`. Each follows the PIMPL design pattern, so for a new
parser or formatter, you only need to define a new implementation (see
[here](../include/hydra_utils/ros_parser.h) for a parsing example, and
[here](../include/hydra_utils/ostream_formatter.h) for a output example).
