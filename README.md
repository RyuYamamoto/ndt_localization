# NDT Localization
3D Localization Package using NDT

## How to use
### 1. Build
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:RyuYamamoto/ndt_localization.git
git clone git@github.com:RyuYamamoto/ndt_omp.git -b ros2-galactic
git clone git@github.com:RyuYamamoto/points_map_loader.git
cd ~/ros2_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2. Run
```bash
source ~/ros2_ws/src/install/setup.bash
ros2 launch ndt_localization ndt_localization.launch.xml map_path:=<POINTCLOUD MAP PATH>
# open other terminal
ros2 bag play <ROSBAG FILE PATH>
```

## related packages
- [ndt_omp](https://github.com/koide3/ndt_omp)  
- [poins_map_loader](https://github.com/RyuYamamoto/points_map_loader)  

[![](https://img.youtube.com/vi/QA-GYnWtqbM/0.jpg)](https://www.youtube.com/watch?v=QA-GYnWtqbM)
