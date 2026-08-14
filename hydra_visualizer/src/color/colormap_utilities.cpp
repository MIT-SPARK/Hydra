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
#include "hydra_visualizer/color/colormap_utilities.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>
#include <glog/logging.h>
#include <spark_dsg/colormaps.h>

#include <fstream>

#include "hydra_visualizer/color/color_parsing.h"

#define REGISTER_CONTINUOUS(type, name) \
  static const auto type##reg =         \
      config::RegistrationWithConfig<ContinuousPalette, type, type::Config>(name)

#define REGISTER_DISCRETE(type, name) \
  static const auto type##reg =       \
      config::RegistrationWithConfig<DiscretePalette, type, type::Config>(name)

namespace hydra::visualizer {

using spark_dsg::Color;

namespace {

REGISTER_CONTINUOUS(GrayPalette, "gray");
REGISTER_CONTINUOUS(QualityPalette, "quality");
REGISTER_CONTINUOUS(IronbowPalette, "ironbow");
REGISTER_CONTINUOUS(RainbowPalette, "rainbow");
REGISTER_CONTINUOUS(SpectrumPalette, "spectrum");
REGISTER_CONTINUOUS(HLSPalette, "hls");
REGISTER_CONTINUOUS(DivergentPalette, "divergent");
REGISTER_DISCRETE(ColorbrewerPalette, "colorbrewer");
REGISTER_DISCRETE(Distinct150Palette, "distinct150");
REGISTER_DISCRETE(ChesapeakeColorPalette, "chesapeake");
REGISTER_DISCRETE(RainbowIdPalette, "rainbow");
REGISTER_DISCRETE(PaletteFromCsvFile, "csv");

static const auto enum_init =
    config::Enum<NamedColors>::Initializer(std::map<NamedColors, std::string>{
        {NamedColors::BLACK, "black"},
        {NamedColors::WHITE, "white"},
        {NamedColors::RED, "red"},
        {NamedColors::GREEN, "green"},
        {NamedColors::BLUE, "blue"},
        {NamedColors::YELLOW, "yellow"},
        {NamedColors::ORANGE, "orange"},
        {NamedColors::PURPLE, "purple"},
        {NamedColors::CYAN, "cyan"},
        {NamedColors::MAGENTA, "magenta"},
        {NamedColors::PINK, "pink"},
        {NamedColors::GRAY, "gray"},
    });

}  // namespace

spark_dsg::Color colorFromName(NamedColors color) {
  switch (color) {
    case NamedColors::BLACK:
      return spark_dsg::Color::black();
    case NamedColors::WHITE:
      return spark_dsg::Color::white();
    case NamedColors::RED:
      return spark_dsg::Color::red();
    case NamedColors::GREEN:
      return spark_dsg::Color::green();
    case NamedColors::BLUE:
      return spark_dsg::Color::blue();
    case NamedColors::YELLOW:
      return spark_dsg::Color::yellow();
    case NamedColors::ORANGE:
      return spark_dsg::Color::orange();
    case NamedColors::PURPLE:
      return spark_dsg::Color::purple();
    case NamedColors::CYAN:
      return spark_dsg::Color::cyan();
    case NamedColors::MAGENTA:
      return spark_dsg::Color::magenta();
    case NamedColors::PINK:
      return spark_dsg::Color::pink();
    case NamedColors::GRAY:
      return spark_dsg::Color::gray();
    default:
      return spark_dsg::Color::black();
  }
}

void fillColorMsg(const Color& color, std_msgs::msg::ColorRGBA& msg) {
  msg.r = static_cast<double>(color.r) / 255.0;
  msg.g = static_cast<double>(color.g) / 255.0;
  msg.b = static_cast<double>(color.b) / 255.0;
}

std_msgs::msg::ColorRGBA makeColorMsg(const Color& color, std::optional<double> alpha) {
  std_msgs::msg::ColorRGBA msg;
  fillColorMsg(color, msg);
  msg.a = alpha.value_or(static_cast<double>(color.a) / 255.0);
  return msg;
}

spark_dsg::Color ContinuousPalette::operator()(double value) const {
  return getColor(value);
}

GrayPalette::GrayPalette(const Config&) {}

spark_dsg::Color GrayPalette::getColor(double value) const {
  return spark_dsg::colormaps::gray(value);
}

void declare_config(GrayPalette::Config&) { config::name("GrayPalette::Config"); }

QualityPalette::QualityPalette(const Config&) {}

spark_dsg::Color QualityPalette::getColor(double value) const {
  return spark_dsg::colormaps::quality(value);
}

void declare_config(QualityPalette::Config&) { config::name("QualityPalette::Config"); }

IronbowPalette::IronbowPalette(const Config&) {}

spark_dsg::Color IronbowPalette::getColor(double value) const {
  return spark_dsg::colormaps::ironbow(value);
}

void declare_config(IronbowPalette::Config&) { config::name("IronbowPalette::Config"); }

RainbowPalette::RainbowPalette(const Config&) {}

spark_dsg::Color RainbowPalette::getColor(double value) const {
  return spark_dsg::colormaps::rainbow(value);
}

void declare_config(RainbowPalette::Config&) { config::name("RainbowPalette::Config"); }

SpectrumPalette::SpectrumPalette() : SpectrumPalette(Config{}) {}

SpectrumPalette::SpectrumPalette(const Config& config)
    : config(config::checkValid(config)) {}

spark_dsg::Color SpectrumPalette::getColor(double value) const {
  return spark_dsg::colormaps::spectrum(value, config.colors);
}

void declare_config(SpectrumPalette::Config& config) {
  using namespace config;
  name("SpectrumPalette::Config");
  field(config.colors, "colors");
}

HLSPalette::HLSPalette() : HLSPalette(Config{}) {}

HLSPalette::HLSPalette(const Config& config)
    : config(config::checkValid(config)),
      hls_start(config.start.toHLS()),
      hls_end(config.end.toHLS()) {}

spark_dsg::Color HLSPalette::getColor(double value) const {
  return spark_dsg::colormaps::hls(value, hls_start, hls_end);
}

void declare_config(HLSPalette::Config& config) {
  using namespace config;
  name("HLSPalette::Config");
  field(config.start, "start");
  field(config.end, "end");
}

DivergentPalette::DivergentPalette() : DivergentPalette(Config{}) {}

DivergentPalette::DivergentPalette(const Config& config)
    : config(config::checkValid(config)) {}

spark_dsg::Color DivergentPalette::getColor(double value) const {
  return spark_dsg::colormaps::divergent(value,
                                         config.hue_low,
                                         config.hue_high,
                                         config.saturation,
                                         config.luminance,
                                         config.dark);
}

void declare_config(DivergentPalette::Config& config) {
  using namespace config;
  name("DivergentPalette::Config");
  field(config.hue_low, "hue_low");
  field(config.hue_high, "hue_high");
  field(config.saturation, "saturation");
  field(config.luminance, "luminance");
  field(config.dark, "dark");
  checkInRange(config.hue_low, 0.0f, 1.0f, "hue_low");
  checkInRange(config.hue_high, 0.0f, 1.0f, "hue_high");
  checkInRange(config.saturation, 0.0f, 1.0f, "saturation");
  checkInRange(config.luminance, 0.0f, 1.0f, "luminance");
}

ColorbrewerPalette::ColorbrewerPalette(const Config&) {}

const std::vector<Color>& ColorbrewerPalette::get() const {
  return spark_dsg::colormaps::colorbrewerPalette();
}

void declare_config(ColorbrewerPalette::Config&) {
  config::name("ColorbrewerPalette::Config");
}

Distinct150Palette::Distinct150Palette(const Config&) {}

const std::vector<Color>& Distinct150Palette::get() const {
  return spark_dsg::colormaps::distinct150Palette();
}

void declare_config(Distinct150Palette::Config&) {
  config::name("Distinct150Palette::Config");
}

ChesapeakeColorPalette::ChesapeakeColorPalette(const Config&) {}

const std::vector<Color>& ChesapeakeColorPalette::get() const {
  return spark_dsg::colormaps::chesapeakePalette();
}

void declare_config(ChesapeakeColorPalette::Config&) {
  config::name("ChesapeakeColorPalette::Config");
}

RainbowIdPalette::RainbowIdPalette(const Config& config)
    : config(config::checkValid(config)) {
  for (size_t i = 0; i < 255; ++i) {
    colors_.push_back(spark_dsg::colormaps::rainbowId(i, config.ids_per_revolution));
  }
}

const std::vector<Color>& RainbowIdPalette::get() const { return colors_; }

void declare_config(RainbowIdPalette::Config& config) {
  using namespace config;
  name("RainbowIdPalette::Config");
  field(config.ids_per_revolution, "ids_per_revolution");
  check(config.ids_per_revolution, GT, 0, "ids_per_revolution");
}

PaletteFromCsvFile::PaletteFromCsvFile(const Config& config)
    : config(config::checkValid(config)) {
  std::ifstream file(config.filepath);
  if (!file.is_open()) {
    LOG(ERROR) << "Could not open csv file '" << config.filepath << "'.";
    return;
  }

  // Read all the data.
  std::string line, word;
  std::vector<std::string> row;
  bool is_header = config.has_header;
  while (std::getline(file, line)) {
    row.clear();
    std::stringstream str(line);
    while (std::getline(str, word, config.separator)) {
      row.push_back(word);
    }

    if (is_header) {
      is_header = false;
      continue;  // TODO(nathan) consider printing header
    }

    const auto red = std::stoi(row[config.rgb_columns[0]]);
    const auto green = std::stoi(row[config.rgb_columns[1]]);
    const auto blue = std::stoi(row[config.rgb_columns[2]]);
    colors_.push_back(Color(red, green, blue));
  }
}

const std::vector<Color>& PaletteFromCsvFile::get() const { return colors_; }

void declare_config(PaletteFromCsvFile::Config& config) {
  using namespace config;
  name("PaletteFromCsvFile::Config");
  field(config.has_header, "has_header");
  field(config.rgb_columns, "rgb_columns");
  field<Path::Absolute>(config.filepath, "filepath");
  check<Path::Exists>(config.filepath, "filepath");
}

RangeColormap::RangeColormap() : RangeColormap(Config{}) {}

RangeColormap::RangeColormap(const Config& config)
    : config(config::checkValid(config)), palette_(config.palette.create()) {}

Color RangeColormap::getColor(double value, double min, double max) const {
  const auto ratio = std::clamp((value - min) / (max - min), 0.0, 1.0);
  return palette_ ? palette_->getColor(ratio) : spark_dsg::colormaps::ironbow(ratio);
}

spark_dsg::Color RangeColormap::operator()(double value, double min, double max) const {
  return getColor(value, min, max);
}

void declare_config(RangeColormap::Config& config) {
  using namespace config;
  name("RangeColormap::Config");
  config.palette.setOptional();
  field(config.palette, "palette");
}

DiscreteColormap::DiscreteColormap() : DiscreteColormap(Config{}) {}

DiscreteColormap::DiscreteColormap(const Config& config)
    : config(config::checkValid(config)), palette_(config.palette.create()) {}

Color DiscreteColormap::getColor(size_t value) const {
  if (!palette_) {
    return spark_dsg::colormaps::rainbowId(value);
  }

  const auto& colors = palette_->get();
  return colors[value % colors.size()];
}

spark_dsg::Color DiscreteColormap::operator()(size_t value) const {
  return getColor(value);
}

void declare_config(DiscreteColormap::Config& config) {
  using namespace config;
  name("DiscreteColormap::Config");
  config.palette.setOptional();
  field(config.palette, "palette");
}

CategoricalColormap::CategoricalColormap() : CategoricalColormap(Config{}) {}

CategoricalColormap::CategoricalColormap(const Config& config)
    : config(config::checkValid(config)), palette_(config.palette.create()) {}

Color CategoricalColormap::getColor(size_t category, size_t total_classes) const {
  if (total_classes && category >= total_classes) {
    return config.default_color;
  }

  if (!palette_) {
    return spark_dsg::colormaps::rainbowId(category);
  }

  const auto& colors = palette_->get();
  if (config.wrap_colors) {
    return colors[category % colors.size()];
  }

  return category < colors.size() ? colors[category] : config.default_color;
}

spark_dsg::Color CategoricalColormap::operator()(size_t category,
                                                 size_t total_classes) const {
  return getColor(category, total_classes);
}

void declare_config(CategoricalColormap::Config& config) {
  using namespace config;
  name("CategoricalColormap::Config");
  config.palette.setOptional();
  field(config.palette, "palette");
  field(config.default_color, "default_color");
  field(config.wrap_colors, "wrap_colors");
}

}  // namespace hydra::visualizer

#undef REGISTER_CONTINUOUS
#undef REGISTER_DISCRETE
