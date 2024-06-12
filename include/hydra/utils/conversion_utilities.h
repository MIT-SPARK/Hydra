#pragma once

#include "hydra/input/input_data.h"

namespace hydra::conversions {

/**
 * @brief make sure that all the images are of the right type
*/
bool normalizeData(InputData& data, bool normalize_labels = true);

bool normalizeDepth(InputData& data);

bool colorToLabels(cv::Mat& label_image, const cv::Mat& colors);

bool convertLabels(InputData& data);

bool convertDepth(InputData& data);

bool convertColor(InputData& data);

}  // namespace hydra::conversions
