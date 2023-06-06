#pragma once
#include <array>
#include <vector>

namespace hydra {

class Colormap {
 public:
  using ColorArray = std::array<float, 3>;

  const ColorArray& operator()(size_t index) const;

  static Colormap Default();

  static Colormap SingleColor();

  Colormap(const std::vector<ColorArray>& colors);

 private:
  std::vector<ColorArray> colors_;
};

}  // namespace hydra
