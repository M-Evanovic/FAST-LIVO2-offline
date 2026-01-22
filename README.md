# FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

## Usage
Config (offline.yaml)
```
common:
    dir_path: "/path/to/your/data/"
    imu_file_name: "imu_data.txt"
    lid_dir_name: "pcd/"
    img_dir_name: "image/"
    save_path: "/path/to/save/pointcloud/"
    camera_config_name: "/path/to/camera/config"
```
## Compile  
```
cd /path/to/workspace
mkdir build
cd build
cmake ..
make
```
## Run
```
# NOTICE: "/path/to/config.yaml" must be set to your config path (./config/offline.yaml)
./fastlivo_mapping /path/to/config.yaml
```

## Acknowledgement
Thank the authors of [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) for open-sourcing their outstanding works.