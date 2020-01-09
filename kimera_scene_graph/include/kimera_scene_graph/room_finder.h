#pragma once

namespace kimera {
class RoomFinder {
 public:
  RoomFinder() = default;
  ~RoomFinder() = default;

 protected:
  void multiPlaneSegmenter() {
    // Set up Organized Multi Plane Segmentation
    // mps.setMinInliers (10000u);
    // mps.setAngularThreshold (pcl::deg2rad (3.0)); // 3 degrees, set as
    // default, well performing for tabletop objects as imaged by a
    // primesense sensor mps.setDistanceThreshold (0.02); // 2cm, set as
    // default, well performing for tabletop objects as imaged by a
    // primesense sensor
  }
};

}  // namespace kimera
