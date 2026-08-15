# Installation

To get started, set up your workspace and clone the Hydra-ROS repo. Then:

```shell
cd <path to ws>/src
vcs import . < hydra_ros/install/packages.yaml
vcs import . < hydra_ros/hydra_plusplus/install/system.repos
rosdep install --from-paths . --ignore-src -r -y

bash src/semantic_inference/install/setup.sh
bash src/hydra_ros/hydra_plusplus/install/setup.sh

cd ..
colcon build --continue-on-error
```

# Running

You may have to install `tmuxp` via `pipx install tmuxp`.

## uHumans2
Make sure you have uHumans2 downloaded. Then:
```
run-hydrapp /data/datasets/uhumans2v2/office_scene/uHumans2_office_s1_00h_v2 -v crisp -p uhumans2.yaml
```
will start Hydra++ using CRISP as the object shape model (make sure to update the path to the uHumans2 bag).

## Kimera-Multi

Make sure you have Kimera-Multi downloaded. Then:
```
run-hydrapp /data/datasets/kimera_multi/hybrid_12_08/12_08_acl_jackal2 acl_jackal2 --bag-start 30 -v crisp -p kimera_multi.yaml
```
will start Hydra++ using CRISP as the object shape model (make sure to update the path to the bag).
