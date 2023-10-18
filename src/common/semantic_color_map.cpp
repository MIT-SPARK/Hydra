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
#include "hydra/common/semantic_color_map.h"

#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <voxblox/core/color.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>

namespace voxblox {

std::ostream& operator<<(std::ostream& out, const Color& color) {
  out << "[" << std::to_string(color.r) << ' ' << std::to_string(color.g) << ' '
      << std::to_string(color.b) << ' ' << std::to_string(color.a) << "]";
  return out;
}

}  // namespace voxblox

using voxblox::Color;

namespace hydra {

size_t ColorHasher::operator()(const Color& k) const {
  // Compute individual hash values for rgb values and combine them using XOR
  // and bit shifting
  return ((std::hash<uint8_t>()(k.r) ^ (std::hash<uint8_t>()(k.g) << 1)) >> 1) ^
         (std::hash<uint8_t>()(k.b) << 1);
}

bool ColorEqual::operator()(const Color& lhs, const Color& rhs) const {
  // note that we don't want to use alpha values
  return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b;
}

SemanticColorMap::SemanticColorMap() : SemanticColorMap(ColorToLabelMap()) {}

SemanticColorMap::SemanticColorMap(const ColorToLabelMap& color_to_label,
                                   const Color& unknown)
    : max_label_(0), color_to_label_(color_to_label), unknown_color_(unknown) {
  for (auto&& [color, label] : color_to_label_) {
    LOG_IF(WARNING, ColorEqual()(unknown_color_, color))
        << "found duplicate color with provided unknown color " << color
        << " for label " << label;
    label_to_color_[label] = color;
    if (label > max_label_) {
      max_label_ = label;
    }
  }
}

std::optional<uint32_t> SemanticColorMap::getLabelFromColor(const Color& color) const {
  const auto it = color_to_label_.find(color);
  if (it != color_to_label_.end()) {
    return it->second;
  }

  if (!unknown_colors_.count(color)) {
    LOG(ERROR) << "Caught an unknown color! RGBA = " << color;
    unknown_colors_.insert(color);
  }

  return std::nullopt;
}

Color SemanticColorMap::getColorFromLabel(const uint32_t& label) const {
  const auto& it = label_to_color_.find(label);
  if (it != label_to_color_.end()) {
    return it->second;
  }

  if (!unknown_labels_.count(label)) {
    LOG(ERROR) << "Caught an unknown label " << std::to_string(label);
    unknown_labels_.insert(label);
  }

  return unknown_color_;
}

size_t SemanticColorMap::getNumLabels() const {
  // note that we want to avoid overflow when max_label hits the maximum
  // semantic label (we return max_label + 1), so this is explicitly size_t
  return static_cast<size_t>(max_label_) + 1;
}

bool SemanticColorMap::isValid() const { return max_label_ != 0; }

std::string SemanticColorMap::toString() const {
  std::stringstream ss;
  ss << "num_labels: " << getNumLabels() << std::endl;
  ss << "invalid_color: " << unknown_color_ << std::endl;
  ss << "color_to_label:" << std::endl;
  for (auto&& [color, label] : color_to_label_) {
    ss << "  - " << color << " -> " << label << std::endl;
  }

  ss << "label_to_color:" << std::endl;
  for (auto&& [label, color] : label_to_color_) {
    ss << "  - " << label << " -> " << color << std::endl;
  }

  return ss.str();
}

SemanticColorMap::Ptr SemanticColorMap::fromCsv(const std::string& filename,
                                                const Color& unknown,
                                                bool skip_first,
                                                char delimiter) {
  std::ifstream file(filename.c_str());
  if (!file.good()) {
    LOG(ERROR) << "Couldn't open file: " << filename.c_str();
    return nullptr;
  }

  ColorToLabelMap cmap;
  size_t row_number = 0;
  std::string curr_line;
  while (std::getline(file, curr_line)) {
    if (skip_first && !row_number) {
      row_number++;
      continue;
    }

    std::stringstream ss(curr_line);
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(ss, column, delimiter)) {
      columns.push_back(column);
      column = "";
    }

    if (columns.size() != 6) {
      LOG(ERROR) << "Row " << row_number << " is invalid: [" << columns << "]";
      return nullptr;
    }

    // We expect the CSV to have header: name, red, green, blue, alpha, id
    const uint8_t r = std::atoi(columns[1].c_str());
    const uint8_t g = std::atoi(columns[2].c_str());
    const uint8_t b = std::atoi(columns[3].c_str());
    const uint8_t a = std::atoi(columns[4].c_str());
    const voxblox::Color rgba(r, g, b, a);
    cmap[rgba] = std::atoi(columns[5].c_str());
    row_number++;
  }

  return std::make_unique<SemanticColorMap>(cmap, unknown);
}

SemanticColorMap::Ptr SemanticColorMap::randomColors(size_t num_labels,
                                                     const Color& unknown) {
  const std::vector<Color> defaults{Color::Gray(),
                                    Color::Green(),
                                    Color::Blue(),
                                    Color::Purple(),
                                    Color::Pink(),
                                    Color::Teal(),
                                    Color::Orange(),
                                    Color::Yellow()};

  ColorToLabelMap cmap;
  for (size_t i = 0; i < num_labels; i++) {
    cmap[(i < defaults.size()) ? defaults.at(i) : voxblox::randomColor()] = i;
  }

  return std::make_unique<SemanticColorMap>(cmap, unknown);
}

std::ostream& operator<<(std::ostream& out, const SemanticColorMap& cmap) {
  return out << cmap.toString();
}

}  // namespace hydra
