#include "hydra/input/input_data.h"

namespace hydra {

bool InputData::hasData() const {
    // we accept data as either a pointcloud (points) or an rgbd image (depth_image)
    // labels or color can encode the labels for a depth image or pointcloud. For the
    // former case, color_image and/or label_image will share the same resolution as
    // depth image and for the latter it will share the same resolution as the
    // pointcloud...
    return (!color_image.empty() || !label_image.empty()) &&
           (!depth_image.empty() || !vertex_map.empty());
}

}  // namespace hydra
