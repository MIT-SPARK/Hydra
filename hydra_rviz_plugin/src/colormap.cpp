#include "hydra_rviz_plugin/colormap.h"

namespace hydra {

const Colormap::ColorArray& Colormap::operator()(size_t index) const {
  return colors_.at(index % colors_.size());
}

Colormap::Colormap(const std::vector<ColorArray>& colors) : colors_(colors) {}

Colormap Colormap::RoomDefault() {
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
  for (const auto& color : colors) {
    std::array<float, 3> new_color;
    new_color[0] = static_cast<float>(color[0]) / 255.0;
    new_color[1] = static_cast<float>(color[1]) / 255.0;
    new_color[2] = static_cast<float>(color[2]) / 255.0;
    float_colors.push_back(new_color);
  }

  return Colormap(float_colors);
}

}  // namespace hydra
