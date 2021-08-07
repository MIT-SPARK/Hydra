DATASET_PATH="$HOME/datasets/uHumans2/"
ROSBAGS=(
  "subway_scene/uHumans2_subway_s1_00h" # 0
  "subway_scene/uHumans2_subway_s1_24h" # 1
  "subway_scene/uHumans2_subway_s1_36h" # 2
  "apartment_scene/uHumans2_apartment_s1_00h" # 3
  "apartment_scene/uHumans2_apartment_s1_01h" # 4
  "apartment_scene/uHumans2_apartment_s1_02h" # 5
  "office_scene/uHumans2_office_s1_00h" # 6
  "office_scene/uHumans2_office_s1_06h" # 7
  "office_scene/uHumans2_office_s1_12h" # 8
  "neighborhood_scene/uHumans2_neighborhood_s1_00h" # 9
  "neighborhood_scene/uHumans2_neighborhood_s1_24h" # 10
  "neighborhood_scene/uHumans2_neighborhood_s1_36h" # 11
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
TYPOLOGY_YAML=(
 "uh2_subway_semantic_label_typology.yaml"
 "uh2_subway_semantic_label_typology.yaml"
 "uh2_subway_semantic_label_typology.yaml"
 "uh2_apartment_semantic_label_typology.yaml"
 "uh2_apartment_semantic_label_typology.yaml"
 "uh2_apartment_semantic_label_typology.yaml"
 "uh2_office_semantic_label_typology.yaml"
 "uh2_office_semantic_label_typology.yaml"
 "uh2_office_semantic_label_typology.yaml"
 "uh2_neighborhood_semantic_label_typology.yaml"
 "uh2_neighborhood_semantic_label_typology.yaml"
 "uh2_neighborhood_semantic_label_typology.yaml"
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


RUNS=(8) # 1 2 3 4 5 6 7 8 9 10 11)
for i in ${RUNS[@]}
do
  BASE_PATH="$ARTIFACTS_PATH/${ROSBAGS[i]}/"

  roslaunch object_db object_db_eval.launch \
    semantic_label_2_color_csv_filename:="${SEGMENTATION_CSVS[i]}" \
    gt_csv_filename:="${OBJECT_CSVS[i]}" \
    scene:="office" \
    target_object_label:="Chair_LeatherOffice" \
  #  &> "object_db_results_${i}.txt"

  ((i++))

done
