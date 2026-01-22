# FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

## Preparation
```
DataFolder  
├ imu_data.txt 
├ pcd  
│  ├ 1767781289679.pcd  
│  ├ 1767781289746.pcd  
│  ├ 1767781289813.pcd  
│  ├ ...  
├ image  
│  ├ 1767781289704.jpg
│  ├ 1767781289771.jpg
│  ├ 1767781289838.jpg
│  ├ ...
```

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
