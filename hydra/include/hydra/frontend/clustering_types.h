#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace hydra::clustering {

using Clusters = std::vector<std::vector<size_t>>;
using LabelIndices = std::map<uint32_t, std::vector<size_t>>;
using LabelSet = std::set<uint32_t>;

std::string printLabels(const LabelSet& labels);

}  // namespace hydra::clustering
