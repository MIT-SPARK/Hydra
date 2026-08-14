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
#pragma once
#include <config_utilities/virtual_config.h>
#include <spark_dsg/color.h>

#include <filesystem>

#include <std_msgs/msg/color_rgba.hpp>

namespace hydra::visualizer {

enum class NamedColors {
  BLACK,
  WHITE,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  ORANGE,
  PURPLE,
  CYAN,
  MAGENTA,
  PINK,
  GRAY,
};

spark_dsg::Color colorFromName(NamedColors color);

void fillColorMsg(const spark_dsg::Color& color, std_msgs::msg::ColorRGBA& msg);

std_msgs::msg::ColorRGBA makeColorMsg(const spark_dsg::Color& color,
                                      std::optional<double> alpha = std::nullopt);

struct ContinuousPalette {
  virtual ~ContinuousPalette() = default;
  virtual spark_dsg::Color getColor(double value) const = 0;
  spark_dsg::Color operator()(double value) const;
};

struct GrayPalette : ContinuousPalette {
  struct Config {};
  explicit GrayPalette(const Config& config = {});
  spark_dsg::Color getColor(double value) const override;
};

void declare_config(GrayPalette::Config& config);

struct QualityPalette : ContinuousPalette {
  struct Config {};
  explicit QualityPalette(const Config& config = {});
  spark_dsg::Color getColor(double value) const override;
};

void declare_config(QualityPalette::Config& config);

struct IronbowPalette : ContinuousPalette {
  struct Config {};
  explicit IronbowPalette(const Config& config = {});
  spark_dsg::Color getColor(double value) const override;
};

void declare_config(IronbowPalette::Config& config);

struct RainbowPalette : ContinuousPalette {
  struct Config {};
  explicit RainbowPalette(const Config& config = {});
  spark_dsg::Color getColor(double value) const override;
};

void declare_config(RainbowPalette::Config& config);

struct SpectrumPalette : ContinuousPalette {
  struct Config {
    std::vector<spark_dsg::Color> colors{spark_dsg::Color(), spark_dsg::Color::red()};
  } const config;

  SpectrumPalette();
  explicit SpectrumPalette(const Config& config);
  spark_dsg::Color getColor(double value) const override;
};

void declare_config(SpectrumPalette::Config& config);

struct HLSPalette : ContinuousPalette {
  struct Config {
    spark_dsg::Color start;
    spark_dsg::Color end = spark_dsg::Color::red();
  } const config;

  HLSPalette();
  explicit HLSPalette(const Config& config);
  spark_dsg::Color getColor(double value) const override;

  const std::array<float, 3> hls_start;
  const std::array<float, 3> hls_end;
};

void declare_config(HLSPalette::Config& config);

struct DivergentPalette : ContinuousPalette {
  struct Config {
    float hue_low = 0.6;
    float hue_high = 0.0;
    float saturation = 0.65;
    float luminance = 0.5;
    bool dark = true;
  } const config;

  DivergentPalette();
  explicit DivergentPalette(const Config& config);
  spark_dsg::Color getColor(double value) const override;
};

void declare_config(DivergentPalette::Config& config);

struct DiscretePalette {
  virtual ~DiscretePalette() = default;
  virtual const std::vector<spark_dsg::Color>& get() const = 0;
};

struct ColorbrewerPalette : DiscretePalette {
  struct Config {};
  ColorbrewerPalette(const Config& config = {});
  const std::vector<spark_dsg::Color>& get() const override;
};

void declare_config(ColorbrewerPalette::Config& config);

struct Distinct150Palette : DiscretePalette {
  struct Config {};
  Distinct150Palette(const Config& config = {});
  const std::vector<spark_dsg::Color>& get() const override;
};

void declare_config(Distinct150Palette::Config& config);

struct ChesapeakeColorPalette : DiscretePalette {
  struct Config {};
  ChesapeakeColorPalette(const Config& config = {});
  const std::vector<spark_dsg::Color>& get() const override;
};

void declare_config(ChesapeakeColorPalette::Config& config);

struct RainbowIdPalette : DiscretePalette {
  struct Config {
    size_t ids_per_revolution = 16;
  } const config;

  RainbowIdPalette();
  RainbowIdPalette(const Config& config);
  const std::vector<spark_dsg::Color>& get() const override;

 private:
  std::vector<spark_dsg::Color> colors_;
};

void declare_config(RainbowIdPalette::Config& config);

struct PaletteFromCsvFile : DiscretePalette {
  struct Config {
    bool has_header = true;
    char separator = ',';
    std::array<size_t, 3> rgb_columns{0, 1, 2};
    std::filesystem::path filepath;
  } const config;

  PaletteFromCsvFile(const Config& config);
  const std::vector<spark_dsg::Color>& get() const override;

 private:
  std::vector<spark_dsg::Color> colors_;
};

void declare_config(PaletteFromCsvFile::Config& config);

///! Colormap for a continuous range of values
struct RangeColormap {
  struct Config {
    //! Color palette to use
    config::VirtualConfig<ContinuousPalette> palette;
  } const config;

  RangeColormap();
  explicit RangeColormap(const Config& config);
  spark_dsg::Color getColor(double value, double min = 0.0, double max = 1.0) const;
  spark_dsg::Color operator()(double value, double min = 0.0, double max = 1.0) const;

 private:
  std::unique_ptr<ContinuousPalette> palette_;
};

void declare_config(RangeColormap::Config& config);

//! Colormap for a unbounded discrete set of values
struct DiscreteColormap {
  struct Config {
    //! Color palette to use
    config::VirtualConfig<DiscretePalette> palette;
  } const config;

  DiscreteColormap();
  explicit DiscreteColormap(const Config& config);
  spark_dsg::Color getColor(size_t value) const;
  spark_dsg::Color operator()(size_t value) const;

 private:
  std::unique_ptr<DiscretePalette> palette_;
};

void declare_config(DiscreteColormap::Config& config);

//! Colormap for a fixed number of categories
struct CategoricalColormap {
  struct Config {
    //! Color palette to use
    config::VirtualConfig<DiscretePalette> palette{Distinct150Palette::Config{}};
    //! Default color to use when the category is unknown or there are no more colors
    spark_dsg::Color default_color = spark_dsg::Color::gray();
    //! Whether or not to reuse colors when there are more categories than colors
    bool wrap_colors = false;
  } const config;

  CategoricalColormap();
  explicit CategoricalColormap(const Config& config);
  spark_dsg::Color getColor(size_t category, size_t total_classes = 0) const;
  spark_dsg::Color operator()(size_t category, size_t total_classes = 0) const;

 private:
  std::unique_ptr<DiscretePalette> palette_;
};

void declare_config(CategoricalColormap::Config& config);

}  // namespace hydra::visualizer
