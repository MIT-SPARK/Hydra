#include "hydra_rviz_plugin/colormap.h"

namespace hydra {

namespace {

void convertColors(std::vector<std::array<uint8_t, 3>>& from,
                   std::vector<std::array<float, 3>>& to) {
  for (const auto& color : from) {
    std::array<float, 3> new_color;
    new_color[0] = static_cast<float>(color[0]) / 255.0;
    new_color[1] = static_cast<float>(color[1]) / 255.0;
    new_color[2] = static_cast<float>(color[2]) / 255.0;
    to.push_back(new_color);
  }
}

}  // namespace

const Colormap::ColorArray& Colormap::operator()(size_t index) const {
  return colors_.at(index % colors_.size());
}

Colormap::Colormap(const std::vector<ColorArray>& colors) : colors_(colors) {}

Colormap Colormap::Default() {
  std::vector<std::array<uint8_t, 3>> colors({
      {166, 206, 227},
      {31, 120, 180},
      {178, 223, 138},
      {51, 160, 44},
      {251, 154, 153},
      {227, 26, 28},
      {253, 191, 111},
      {255, 127, 0},
      {202, 178, 214},
      {106, 61, 154},
      {255, 255, 153},
      {177, 89, 40},
  });
  std::vector<ColorArray> float_colors;
  convertColors(colors, float_colors);
  return Colormap(float_colors);
}

Colormap Colormap::SingleColor() {
  // TODO(nathan) pick something better
  std::vector<std::array<uint8_t, 3>> colors({
      {255, 0, 0},
      {0, 255, 0},
      {255, 0, 0},
  });
  std::vector<ColorArray> float_colors;
  convertColors(colors, float_colors);
  return Colormap(float_colors);
}

}  // namespace hydra
