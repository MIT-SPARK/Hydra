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
SEGMENTATION_CSVS=(
  "tesse_multiscene_underground1_segmentation_mapping.csv"
  "tesse_multiscene_underground1_segmentation_mapping.csv"
  "tesse_multiscene_underground1_segmentation_mapping.csv"
  "tesse_multiscene_archviz1_segmentation_mapping.csv"
  "tesse_multiscene_archviz1_segmentation_mapping.csv"
  "tesse_multiscene_archviz1_segmentation_mapping.csv"
  "tesse_multiscene_office2_segmentation_mapping.csv"
  "tesse_multiscene_office2_segmentation_mapping.csv"
  "tesse_multiscene_office2_segmentation_mapping.csv"
  "tesse_multiscene_neighborhood1_segmentation_mapping.csv"
  "tesse_multiscene_neighborhood1_segmentation_mapping.csv"
  "tesse_multiscene_neighborhood1_segmentation_mapping.csv"
)
OBJECT_CSVS=(
  "subway_static_tfs_ros.csv"
  "subway_static_tfs_ros.csv"
  "subway_static_tfs_ros.csv"
  "apartment_static_tfs_ros.csv"
  "apartment_static_tfs_ros.csv"
  "apartment_static_tfs_ros.csv"
  "office_static_tfs_ros.csv"
  "office_static_tfs_ros.csv"
  "office_static_tfs_ros.csv"
  "neighborhood_static_tfs_ros.csv"
  "neighborhood_static_tfs_ros.csv"
  "neighborhood_static_tfs_ros.csv"
)
ARTIFACTS_PATH="$HOME/Documents/uHumans2_VIO_vxblx/"


RUNS=(0) # 1 2 3 4 5 6 7 8 9 10 11)
for i in ${RUNS[@]}
do
  BASE_PATH="$ARTIFACTS_PATH/${ROSBAGS[i]}/"

  roslaunch kimera_scene_graph kimera_scene_graph_eval.launch \
    semantic_label_2_color_csv_filename:="${SEGMENTATION_CSVS[i]}" \
    tsdf_map_path:="$BASE_PATH/tsdf_esdf_DVIO.vxblx" \
    esdf_map_path:="$BASE_PATH/tsdf_esdf_DVIO.vxblx" \
    sparse_graph_path:="$BASE_PATH/sparse_graph_DVIO.vxblx" \
  &> kimera_scene_graph_out.txt &

  roslaunch object_db object_db_eval.launch \
    semantic_label_2_color_csv_filename:="${SEGMENTATION_CSVS[i]}" \
    gt_csv_filename:="${OBJECT_CSVS[i]}"

  ((i++))
done
