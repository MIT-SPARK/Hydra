#include <sstream>
#include <fstream>
#include <iostream>
#include <memory>
#include <cstring>

#include "object_db/ply_io.h"

#define TINYPLY_IMPLEMENTATION
#include "object_db/tinyply.h"

namespace object_registration {

int PLYReader::read(const std::string& file_name,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::unique_ptr<std::istream> file_stream;
  try {
    file_stream.reset(new std::ifstream(file_name, std::ios::binary));

    if (!file_stream || file_stream->fail()) {
      std::cerr << "Failed to open " << file_name << std::endl;
      return -1;
    }

    tinyply::PlyFile file;
    file.parse_header(*file_stream);

    std::shared_ptr<tinyply::PlyData> vertices;

    try {
      vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
      return -1;
    }

    file.read(*file_stream);

    if (vertices) {
      std::cout << "\tRead " << vertices->count << " total vertices " << std::endl;
      if (vertices->t == tinyply::Type::FLOAT32) {
        std::vector<float3> verts_floats(vertices->count);
        const size_t numVerticesBytes = vertices->buffer.size_bytes();
        std::memcpy(verts_floats.data(), vertices->buffer.get(), numVerticesBytes);
        for (auto& i : verts_floats) {
          cloud->push_back({i.x, i.y, i.z});
        }
      }
      if (vertices->t == tinyply::Type::FLOAT64) {
        std::vector<double3> verts_doubles(vertices->count);
        const size_t numVerticesBytes = vertices->buffer.size_bytes();
        std::memcpy(verts_doubles.data(), vertices->buffer.get(), numVerticesBytes);
        for (auto& i : verts_doubles) {
          pcl::PointXYZ point{static_cast<float>(i.x), static_cast<float>(i.y),
                              static_cast<float>(i.z)};
          cloud->push_back(point);
        }
      }
    }

  } catch (const std::exception& e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

}  // namespace object_registration
