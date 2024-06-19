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

#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>

#include "hydra/utils/csv_reader.h"

namespace hydra {

SemanticColorMap::SemanticColorMap() : SemanticColorMap(ColorToLabelMap()) {}

SemanticColorMap::SemanticColorMap(const ColorToLabelMap& color_to_label,
                                   const Color& unknown)
    : max_label_(0), color_to_label_(color_to_label), unknown_color_(unknown) {
  for (auto&& [color, label] : color_to_label_) {
    LOG_IF(WARNING, unknown_color_ == color)
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
    LOG(ERROR) << "Caught an unknown color " << color << ".";
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
                                                char delimiter,
                                                bool skip_first_line) {
  // Required headers (r,g,b,a,id) only
  static const std::string red_header = "red";
  static const std::string green_header = "green";
  static const std::string blue_header = "blue";
  static const std::string alpha_header = "alpha";
  static const std::string id_header = "id";

  const CsvReader reader(filename, delimiter, skip_first_line);
  if (!reader) {
    return nullptr;
  }

  if (!reader.checkRequiredHeaders({red_header, green_header, blue_header, id_header})) {
    return nullptr;
  }

  ColorToLabelMap cmap;
  const bool has_alpha = reader.hasHeader(alpha_header);
  for (size_t row = 0; row < reader.numRows(); row++) {
    const int id = std::stoi(reader.getEntry(id_header, row));
    const uint8_t r = std::stoi(reader.getEntry(red_header, row));
    const uint8_t g = std::stoi(reader.getEntry(green_header, row));
    const uint8_t b = std::stoi(reader.getEntry(blue_header, row));
    const uint8_t a = has_alpha ? std::stoi(reader.getEntry(alpha_header, row)) : 255;
    cmap[Color(r, g, b, a)] = id;
  }

  return std::make_unique<SemanticColorMap>(cmap, unknown);
}

SemanticColorMap::Ptr SemanticColorMap::randomColors(size_t num_labels,
                                                     const Color& unknown) {
  const std::vector<Color> defaults{Color::gray(),
                                    Color::green(),
                                    Color::blue(),
                                    Color::purple(),
                                    Color::magenta(),
                                    Color::cyan(),
                                    Color::orange(),
                                    Color::yellow(),
                                    Color::pink()};

  ColorToLabelMap cmap;
  for (size_t i = 0; i < num_labels; i++) {
    cmap[(i < defaults.size()) ? defaults.at(i) : Color::random()] = i;
  }

  return std::make_unique<SemanticColorMap>(cmap, unknown);
}

std::ostream& operator<<(std::ostream& out, const SemanticColorMap& cmap) {
  return out << cmap.toString();
}

}  // namespace hydra
