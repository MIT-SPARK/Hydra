#pragma once

//Forward declare to avoid includes.
namespace cv {

class Mat;

}  // namespace cv
namespace hydra {

// Forward declare to avoid includes.
struct InputData;
struct InputPacket;

namespace conversions {

bool inputPacketToData(InputData& input_data, const InputPacket& input_packet);

/**
 * @brief make sure that all the images are of the right type
*/
bool normalizeData(InputData& data, bool normalize_labels = true);

bool normalizeDepth(InputData& data);

bool colorToLabels(cv::Mat& label_image, const cv::Mat& colors);

bool convertLabels(InputData& data);

bool convertDepth(InputData& data);

bool convertColor(InputData& data);

}  // namespace conversions

}  // namespace hydra
