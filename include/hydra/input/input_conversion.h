#pragma once

#include <memory>

#include "hydra/input/input_data.h"

namespace hydra {

struct InputPacket;

namespace conversions {

/**
 * @brief Convert an input packet to a valid input data format if possible.
 * @param input_packet The input packet to convert.
 * @param vertices_in_world_frame If true, convert the vertex image to be in world
 * frame. Otherwise it will be in sensor frame.
 * @param normalize_labels Force label normalization
 * @return The input data if successful, nullptr otherwise.
 */
std::unique_ptr<InputData> parseInputPacket(const InputPacket& input_packet,
                                            bool vertices_in_world_frame = false,
                                            bool normalize_labels = true);

/**
 * @brief Ensure that all the images are of the right type
 * @param data Input data to normalize
 * @param normalize_labels Whether or not to ensure labels are the correct type
 * @return Whether or not the input normalization was successful
 */
bool normalizeData(InputData& data, bool normalize_labels = true);

/**
 * @brief Ensure that the vertex map is in the correct frame.
 * @param data The input data to convert.
 * @param in_world_frame If true, the vertex map will be converted to world frame.
 * Otherwise it will be in sensor frame.
 */
void convertVertexMap(InputData& data, bool in_world_frame);

}  // namespace conversions
}  // namespace hydra
