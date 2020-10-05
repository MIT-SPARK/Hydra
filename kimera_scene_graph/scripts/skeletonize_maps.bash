DATASET_PATH="$HOME/datasets/uHumans2/"
ROSBAGS=(
  "subway_scene/uHumans2_subway_s1_00h"
  "subway_scene/uHumans2_subway_s1_24h"
  "subway_scene/uHumans2_subway_s1_36h"
  "apartment_scene/uHumans2_apartment_s1_00h"
  "apartment_scene/uHumans2_apartment_s1_01h"
  "apartment_scene/uHumans2_apartment_s1_02h"
  "office_scene/uHumans2_office_s1_00h"
  "office_scene/uHumans2_office_s1_06h"
  "office_scene/uHumans2_office_s1_12h"
  "neighborhood_scene/uHumans2_neighborhood_s1_00h"
  "neighborhood_scene/uHumans2_neighborhood_s1_24h"
  "neighborhood_scene/uHumans2_neighborhood_s1_36h"
)
ARTIFACTS_PATH="$HOME/Documents/uHumans2_VIO/"

i=0
for ROSBAG in ${ROSBAGS[@]}
do
  BASE_PATH="$ARTIFACTS_PATH/$ROSBAG/"

  roslaunch voxblox_skeleton skeletonize_map.launch \
    base_path:=$BASE_PATH \
    input_map_name:="tsdf_esdf_DVIO.vxblx" \
    output_map_name:="skeleton_DVIO.vxblx" \
    sparse_graph_name:="sparse_graph_DVIO.vxblx"

  roslaunch voxblox_skeleton skeletonize_map.launch \
    base_path:=$BASE_PATH \
    input_map_name:="tsdf_esdf_gt.vxblx" \
    output_map_name:="skeleton_gt.vxblx" \
    sparse_graph_name:="sparse_graph_gt.vxblx"

  ((i++))
done
