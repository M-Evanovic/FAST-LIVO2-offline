/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

LIVMapper::LIVMapper(std::string _config_dir)
    : extT(0, 0, 0), extR(M3D::Identity()), config_dir(_config_dir) {
  config_path = config_dir + "offline.yaml";
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(config_path);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(config_path, voxel_config);

  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  initializeFiles();
  initializeComponents();
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(const std::string config_path) {
  auto yaml = YAML::LoadFile(config_path);

  dir_path = yaml["common"]["dir_path"].as<std::string>();
  imu_file_name = dir_path + yaml["common"]["imu_file_name"].as<std::string>();
  lid_dir_name = dir_path + yaml["common"]["lid_dir_name"].as<std::string>();
  img_dir_name = dir_path + yaml["common"]["img_dir_name"].as<std::string>();
  std::cout << "IMU FILE PATH: " << imu_file_name << std::endl;
  std::cout << "LIDAR DIR PATH: " << lid_dir_name << std::endl;
  std::cout << "IMG DIR PATH: " << img_dir_name << std::endl;
  save_path = yaml["common"]["save_path"].as<std::string>();
  save_epoch = yaml["common"]["save_epoch"].as<int>();

  imu_en = yaml["common"]["imu_en"].as<bool>();
  img_en = yaml["common"]["img_en"].as<int>();
  lidar_en = yaml["common"]["lidar_en"].as<int>();

  std::string camera_config_name = yaml["common"]["camera_config_name"].as<std::string>();
  camera_config_path = config_dir + camera_config_name;

  normal_en = yaml["vio"]["normal_en"].as<bool>();
  inverse_composition_en = yaml["vio"]["inverse_composition_en"].as<bool>();
  max_iterations = yaml["vio"]["max_iterations"].as<int>();
  IMG_POINT_COV = yaml["vio"]["img_point_cov"].as<double>();
  raycast_en = yaml["vio"]["raycast_en"].as<bool>();
  exposure_estimate_en = yaml["vio"]["exposure_estimate_en"].as<bool>();
  inv_expo_cov = yaml["vio"]["inv_expo_cov"].as<double>();
  grid_size = yaml["vio"]["grid_size"].as<int>();
  grid_n_height = yaml["vio"]["grid_n_height"].as<int>();
  patch_pyrimid_level = yaml["vio"]["patch_pyrimid_level"].as<int>();
  patch_size = yaml["vio"]["patch_size"].as<int>();
  outlier_threshold = yaml["vio"]["outlier_threshold"].as<double>();

  exposure_time_init = yaml["time_offset"]["exposure_time_init"].as<double>();
  lid_time_interval = yaml["time_offset"]["lid_time_interval"].as<double>();
  img_time_offset = yaml["time_offset"]["img_time_offset"].as<double>();
  imu_time_offset = yaml["time_offset"]["imu_time_offset"].as<double>();
  lidar_time_offset = yaml["time_offset"]["lidar_time_offset"].as<double>();
  imu_prop_enable = yaml["uav"]["imu_rate_odom"].as<bool>();
  gravity_align_en = yaml["uav"]["gravity_align_en"].as<bool>();

  gyr_cov = yaml["imu"]["gyr_cov"].as<double>();
  acc_cov = yaml["imu"]["acc_cov"].as<double>();
  b_gyr_cov = yaml["imu"]["b_gyr_cov"].as<double>();
  b_acc_cov = yaml["imu"]["b_acc_cov"].as<double>();
  imu_int_frame = yaml["imu"]["imu_int_frame"].as<int>();
  gravity_est_en = yaml["imu"]["gravity_est_en"].as<bool>();
  ba_bg_est_en = yaml["imu"]["ba_bg_est_en"].as<bool>();
  
  p_pre->blind = yaml["preprocess"]["blind"].as<double>();
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
  filter_size_surf_min = yaml["preprocess"]["filter_size_surf"].as<double>();
  p_pre->lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
  p_pre->N_SCANS = yaml["preprocess"]["scan_line"].as<int>();
  p_pre->point_filter_num = yaml["preprocess"]["point_filter_num"].as<int>();
  p_pre->feature_enabled = yaml["preprocess"]["feature_extract_enabled"].as<bool>();

  pcd_save_interval = yaml["pcd_save"]["interval"].as<int>();
  pcd_save_en = yaml["pcd_save"]["pcd_save_en"].as<bool>();
  pcd_save_type = yaml["pcd_save"]["pcd_save_type"].as<int>();
  img_save_interval = yaml["image_save"]["interval"].as<int>();
  img_save_en = yaml["image_save"]["img_save_en"].as<bool>();
  colmap_output_en = yaml["pcd_save"]["colmap_output_en"].as<bool>();
  filter_size_pcd = yaml["pcd_save"]["filter_size_pcd"].as<double>();
  extrinT = yaml["extrin_calib"]["extrinsic_T"].as<vector<double>>();
  extrinR = yaml["extrin_calib"]["extrinsic_R"].as<vector<double>>();
  cameraextrinT = yaml["extrin_calib"]["Pcl"].as<vector<double>>();
  cameraextrinR = yaml["extrin_calib"]["Rcl"].as<vector<double>>();
  plot_time = yaml["debug"]["plot_time"].as<double>();
  frame_cnt = yaml["debug"]["frame_cnt"].as<int>();

  blind_rgb_points = yaml["publish"]["blind_rgb_points"].as<double>();
  pub_scan_num = yaml["publish"]["pub_scan_num"].as<int>();
  dense_map_en = yaml["publish"]["dense_map_en"].as<bool>();
}

void LIVMapper::initializeComponents() {
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::LoadFromCamearaConfig(camera_config_path, vio_manager->cam))
    throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
  p_imu->set_imu_init_frame_num(imu_int_frame);
  p_imu->set_lid_time_interval(lid_time_interval);

  if (!imu_en)
    p_imu->disable_imu();
  if (!gravity_est_en)
    p_imu->disable_gravity_est();
  if (!ba_bg_est_en)
    p_imu->disable_bias_est();
  if (!exposure_estimate_en)
    p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() {
  if (pcd_save_en && colmap_output_en) {
    const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
    
    std::string chmodCommand = "chmod +x " + folderPath;
    
    int chmodRet = system(chmodCommand.c_str());  
    if (chmodRet != 0) {
        std::cerr << "Failed to set execute permissions for the script." << std::endl;
        return;
    }

    int executionRet = system(folderPath.c_str());
    if (executionRet != 0) {
        std::cerr << "Failed to execute the script." << std::endl;
        return;
    }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/all_pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/all_image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::handleFirstFrame() {
  if (!is_first_frame) {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() {
  if (!p_imu->imu_need_init && !gravity_align_finished) {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() {
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en)
    gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() <<
  // std::endl; std::cout << "[ Mapping ] predict cov: " <<
  // _state.cov.diagonal().transpose() << std::endl; std::cout << "[ Mapping ]
  // predict sta: " << state_propagat.pos_end.transpose() <<
  // state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() {
  switch (LidarMeasures.lio_vio_flg) {
  case VIO:
    handleVIO();
    break;
  case LIO:
  case LO:
    handleLIO();
    break;
  }
}

void LIVMapper::handleVIO() {
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20)
           << LidarMeasures.last_lio_update_time - _first_lidar_time << " "
           << euler_cur.transpose() * 57.3 << " " << _state.pos_end.transpose()
           << " " << _state.vel_end.transpose() << " "
           << _state.bias_g.transpose() << " " << _state.bias_a.transpose()
           << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;

  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }

  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size()
            << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) -
           plot_time) < (frame_cnt / 2 * 0.1)) {
    vio_manager->plot_flag = true;
  } else {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(
      LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_,
      LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  publish_frame_world(vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20)
           << LidarMeasures.last_lio_update_time - _first_lidar_time << " "
           << euler_cur.transpose() * 57.3 << " " << _state.pos_end.transpose()
           << " " << _state.vel_end.transpose() << " "
           << _state.bias_g.transpose() << " " << _state.bias_a.transpose()
           << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " "
           << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() {
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time
           << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose()
           << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " "
           << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;

  if (feats_undistort->empty() || (feats_undistort == nullptr)) {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);

  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body,
                 feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  if (!lidar_map_inited) {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  euler_cur = RotMtoEuler(_state.rot_end);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x,
        world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) *
              (-point_crossmat).transpose() +
          _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;

  double t4 = omp_get_wtime();

  if (voxelmap_manager->config_setting_.map_sliding_en) {
    voxelmap_manager->mapSliding();
  }

  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort
                                                     : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                        &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(vio_manager);

  frame_num++;
  aver_time_consu =
      aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                 "
         "   |\033[0m\n");
  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage",
         "Time (secs)");
  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time",
         aver_time_consu);
  printf("\033[1;34m+----------------------------------------------------------"
         "---+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20)
           << LidarMeasures.last_lio_update_time - _first_lidar_time << " "
           << euler_cur.transpose() * 57.3 << " " << _state.pos_end.transpose()
           << " " << _state.vel_end.transpose() << " "
           << _state.bias_g.transpose() << " " << _state.bias_a.transpose()
           << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " "
           << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() {
  if (pcd_save_en &&
      (pcl_wait_save->points.size() > 0 ||
       pcl_wait_save_intensity->points.size() > 0) &&
      pcd_save_interval < 0) {
    std::string raw_points_dir = std::string(ROOT_DIR) + save_path + "all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + save_path + "all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);

      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;

      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if (colmap_output_en) {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) {
          const auto &point = downsampled_cloud->points[i];
          fout_points << i << " " << std::fixed << std::setprecision(6)
                      << point.x << " " << point.y << " " << point.z << " "
                      << static_cast<int>(point.r) << " "
                      << static_cast<int>(point.g) << " "
                      << static_cast<int>(point.b) << " " << 0 << std::endl;
        }
      }
    } else {
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run() {
  LoadAllFiles();
  int epoch = 0;
  while (!imu_buffer.empty() 
        && !lid_raw_data_buffer.empty() 
        && !img_buffer.empty() ) {
    if (!sync_packages(LidarMeasures)) {
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();

    if (save_epoch > 0 && epoch++ == save_epoch) {
      savePCD();
      return;
    }
  }
  savePCD();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot,
                               const Eigen::Vector3d t,
                               const PointCloudXYZI::Ptr &input_cloud,
                               PointCloudXYZI::Ptr &trans_cloud) {
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++) {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po) {
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T>
void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi,
                                 Matrix<T, 3, 1> &po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T>
Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi) {
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi,
                                    PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LIVMapper::LoadAllFiles() {
  if (lidar_en)
    LoadPCD(lid_dir_name);
  if (imu_en)
    LoadIMU(imu_file_name);
  if (img_en)
    LoadImage(img_dir_name);
}

void LIVMapper::LoadIMU(const std::string imu_file_name) {
  double last_sec = 0.0;

  std::ifstream imu_file(imu_file_name);
  if (!imu_file.is_open()) {
    std::cerr << "Fail to open IMU file: " << imu_file_name << std::endl;
    return;
  }
    
  std::string line;
  bool first_line = true;

  while (std::getline(imu_file, line)) {
    if (line.find("sys_time_ms") != std::string::npos || line.empty()) {
      continue;
    }

    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> parts;
    
    while (std::getline(iss, token, ',')) {
        parts.push_back(token);
    }
    
    if (parts.size() < 8) {
        continue;
    }

    double sys_time_ns = 0.0;
    try {
        sys_time_ns = std::stod(parts[1]);
    } catch (...) {
        std::cerr << "IMU 行解析时间戳失败, 原始行: " << line << std::endl;
        continue;
    }
    double sec = sys_time_ns / 1e9 + imu_time_offset;

    if (sec < last_sec) {
        std::cerr << "IMU data loop back, sec=" << sec 
                  << ", last sec=" << last_sec << "，跳过该条数据" << std::endl;
        continue;
    }
    last_sec = sec;
    if (!std::isfinite(sec) || sec < 0.0) {
        std::cerr << "IMU 时间戳异常: sys_time_ns=" << sys_time_ns
                  << ", sec=" << sec << "，强制置 0" << std::endl;
        sec = 0.0;
    }
    uint32_t sec_uint = static_cast<uint32_t>(sec);
    uint32_t nsec_uint = static_cast<uint32_t>((sec - sec_uint) * 1e9);

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu());
    imu_msg->header.stamp.sec = sec_uint;
    imu_msg->header.stamp.nsec = nsec_uint;
    imu_msg->header.frame_id = "imu_link";

    imu_msg->linear_acceleration.x = std::stod(parts[2]);
    imu_msg->linear_acceleration.y = std::stod(parts[3]);
    imu_msg->linear_acceleration.z = std::stod(parts[4]);

    imu_msg->angular_velocity.x = M_PI * std::stod(parts[5]) / 180.0 ;
    imu_msg->angular_velocity.y = M_PI * std::stod(parts[6]) / 180.0 ;
    imu_msg->angular_velocity.z = M_PI * std::stod(parts[7]) / 180.0 ;

    imu_buffer.push_back(imu_msg);
  }

  imu_file.close();

  std::cout << "Loaded " << imu_buffer.size() << " IMU messages" << std::endl;
}

// void LIVMapper::LoadPCD(const std::string lid_dir_name) {
//   double last_sec = 0.0;

//   DIR* lid_dir = opendir(lid_dir_name.c_str());
//   if (!lid_dir) {
//     std::cerr << "Fail to load LIDAR dir: " << lid_dir_name << std::endl;
//     return;
//   }

//   std::vector<std::string> pcd_files;
//   struct dirent* entry;
  
//   while ((entry = readdir(lid_dir)) != nullptr) {
//       std::string filename = entry->d_name;
//       if (filename.length() > 4 && 
//           filename.substr(filename.length() - 4) == ".pcd") {
//           pcd_files.push_back(filename);
//       }
//   }
//   closedir(lid_dir);
  
//   std::sort(pcd_files.begin(), pcd_files.end());
//   for (const auto& pcd_name : pcd_files) {
//     std::string ts_str = pcd_name.substr(0, pcd_name.length() - 4);
//     PointCloudXYZI::Ptr raw_cloud_ptr(new PointCloudXYZI());
//     std::string pcd_path = lid_dir_name + pcd_name;
//     if (pcl::io::loadPCDFile<PointType>(pcd_path, *raw_cloud_ptr) == 0) {
//       std::cout << raw_cloud_ptr->points.size() << " points loaded from " << pcd_path << std::endl;
//       double sec_end = std::stod(ts_str) / 1000.0 + lidar_time_offset;
//       if (sec_end <= last_sec) {
//         std::cerr << "LIDAR data time loop back: "
//                   << "current sec_end = " << sec_end 
//                   << ", last_sec = " << last_sec << std::endl;
//         continue;
//       }
//       // uint32_t sec_uint_end = static_cast<uint32_t>(sec_end);
//       // uint32_t nsec_uint_end = static_cast<uint32_t>((sec_end - sec_uint_end) * 1e9);

//       lid_raw_data_buffer.push_back(raw_cloud_ptr);
//       lid_header_time_buffer.push_back(sec_end);

//       last_sec = sec_end;
//     }
//   }
//   std::cout << "Loaded " << lid_raw_data_buffer.size() << " PCD files" << std::endl;
// }

using PointXYZIT = lx_ros::LxPointXYZIT;
void LIVMapper::LoadPCD(const std::string lid_dir_name) {
  DIR* dir = opendir(lid_dir_name.c_str());
  if (!dir) {
      std::cerr << "Fail to load PCD dir: " << lid_dir_name << std::endl;
      return;
  }

  std::vector<std::string> pcd_files;
  struct dirent* entry;
  
  while ((entry = readdir(dir)) != nullptr) {
      std::string filename = entry->d_name;
      if (filename.length() > 4 && 
          filename.substr(filename.length() - 4) == ".pcd") {
          pcd_files.push_back(filename);
      }
  }
  closedir(dir);
  
  std::sort(pcd_files.begin(), pcd_files.end());

  double last_timestamp_lidar = -1.0;

  for (const auto& pcd_name : pcd_files) {
    std::string pcd_path = lid_dir_name + "/" + pcd_name;
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());

    std::ifstream file(pcd_path, std::ios::binary);
    if (!file.is_open()) {
      std::cerr << "Failed to open PCD: " << pcd_path << std::endl;
      continue;
    }

    std::string line;
    size_t num_points = 0;
    bool header_end = false;

    while (std::getline(file, line)) {
      if (line.find("POINTS") == 0) {
        std::istringstream iss(line);
        std::string tag;
        iss >> tag >> num_points;
      }
      if (line.find("DATA binary") == 0) {
        header_end = true;
        break;
      }
    }

    if (!header_end || num_points == 0) {
      std::cerr << "Invalid PCD header\n";
      continue;
    }

    std::vector<RawPoint28> raw(num_points);
    file.read(reinterpret_cast<char*>(raw.data()),
              num_points * sizeof(RawPoint28));
    size_t read_cnt = file.gcount() / sizeof(RawPoint28);
    file.close();

    if (read_cnt == 0) continue;

    const double time_head = raw[0].timestamp * 1e-6 + lidar_time_offset;
    if (time_head < last_timestamp_lidar) {
      std::cerr << "LiDAR data loop back" << std::endl;
      continue;
    }
    last_timestamp_lidar = time_head;

    cloud->reserve(read_cnt / p_pre->point_filter_num + 1);

    for (size_t i = 0; i < read_cnt; ++i) {
      if (i % p_pre->point_filter_num != 0)
        continue;

      const auto& rp = raw[i];

      const double x = rp.x / 1000.0;
      const double y = rp.y / 1000.0;
      const double z = rp.z / 1000.0;

      const double dist_sqr = x * x + y * y + z * z;
      if (dist_sqr < p_pre->blind_sqr ||
          !std::isfinite(x) ||
          !std::isfinite(y) ||
          !std::isfinite(z))
        continue;

      PointType pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      pt.intensity = static_cast<float>(rp.intensity);

      pt.normal_x = 0;
      pt.normal_y = 0;
      pt.normal_z = 0;

      const double t = rp.timestamp * 1e-6;
      pt.curvature = (t - time_head) * 1e3;

      cloud->points.push_back(pt);
    }

    std::sort(cloud->points.begin(), cloud->points.end(),
              [](const PointType& a, const PointType& b) {
                return a.curvature < b.curvature;
              });

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = true;
    
    lid_raw_data_buffer.push_back(cloud);
    lid_header_time_buffer.push_back(time_head);
  }
}

void LIVMapper::LoadImage(const std::string img_dir_name) {
  double last_sec = 0.0;

  DIR* img_dir = opendir(img_dir_name.c_str());
  if (!img_dir) {
    std::cerr << "Fail to load IMAGE dir: " << img_dir_name << std::endl;
    return;
  }

  std::vector<std::string> img_files;
    struct dirent* entry;
    
    while ((entry = readdir(img_dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename.length() > 4 && 
            filename.substr(filename.length() - 4) == ".jpg") {
            img_files.push_back(filename);
        }
    }
    closedir(img_dir);

    std::sort(img_files.begin(), img_files.end());

    for (const auto &img_name : img_files) {
      std::string ts_str = img_name.substr(0, img_name.length() - 4);
      // double sec = std::stod(ts_str) / 1000.0;
      // uint32_t sec_uint = static_cast<uint32_t>(sec);
      // uint32_t nsec_uint = static_cast<uint32_t>((sec - sec_uint) * 1e9);

      std::string img_path = img_dir_name + img_name;
      cv::Mat cv_img = cv::imread(img_path, cv::IMREAD_COLOR);

      if (!cv_img.empty()) {
        double sec_end = std::stod(ts_str) / 1000.0 + img_time_offset;
        // uint32_t sec_uint_end = static_cast<uint32_t>(sec_end);
        // uint32_t nsec_uint_end = static_cast<uint32_t>((sec_end - sec_uint_end) * 1e9);
        if (sec_end <= last_sec) {
          std::cerr << "Camera data time loop back: "
                    << "current sec_end = " << sec_end 
                    << ", last_sec = " << last_sec << std::endl;
          continue;
        }

        img_buffer.push_back(cv_img);
        img_time_buffer.push_back(sec_end);

        last_sec = sec_end;
      }
    }
    
    std::cout << "Loaded " << img_buffer.size() << " images" << std::endl;
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas) {
  if (lid_raw_data_buffer.empty() && lidar_en)
    return false;
  if (img_buffer.empty() && img_en)
    return false;
  if (imu_buffer.empty() && imu_en)
    return false;

  switch (slam_mode_) {
  case ONLY_LIO: {
    if (meas.last_lio_update_time < 0.0)
      meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed) {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1)
        return false;

      meas.lidar_frame_beg_time =
          lid_header_time_buffer.front(); // generate lidar_frame_beg_time
      double lidar_time_interval =
          meas.lidar->points.back().curvature / double(1000);
      if (lid_time_interval != 0) {
        lidar_time_interval = lid_time_interval;
      }
      meas.lidar_frame_end_time =
          meas.lidar_frame_beg_time +
          lidar_time_interval; // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true; // flag
    }

    if (imu_en &&
        last_timestamp_imu <
            meas.lidar_frame_end_time) { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty()) {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time)
        break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO: {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg) {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO: {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0)
        meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lidar_time_interval =
          lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      if (lid_time_interval != 0) {
        lidar_time_interval = lid_time_interval;
      }
      double lid_newest_time =
          lid_header_time_buffer.back() + lidar_time_interval;
      double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

      if (img_capture_time < meas.last_lio_update_time + 0.00001) {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        std::cerr << "[ Data Cut ] Throw one image frame! " << std::endl;
        return false;
      }

      if (img_capture_time > lid_newest_time ||
          img_capture_time > imu_newest_time) {
        std::cerr << "[ Data Cut ] lost first camera frame" << std::endl;
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty()) {
        if (imu_buffer.front()->header.stamp.toSec() > m.lio_time)
          break;

        if (imu_buffer.front()->header.stamp.toSec() >
            meas.last_lio_update_time)
          m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty()) {
        if (lid_header_time_buffer.front() > img_capture_time)
          break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++) {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms) {
            pt.curvature +=
                (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          } else {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO: {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = imu_buffer.front()->header.stamp.toSec();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front()->header.stamp.toSec();
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   imu_buffer.front()->header.stamp.toSec());
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed =
          false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default: {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO: {
    if (!lidar_pushed) {
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())
        return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time =
          lid_header_time_buffer.front(); // generate lidar_beg_time
      double lidar_time_interval =
          meas.lidar->points.back().curvature / double(1000);
      if (lid_time_interval != 0) {
        lidar_time_interval = lid_time_interval;
      }
      meas.lidar_frame_end_time =
          meas.lidar_frame_beg_time +
          lidar_time_interval; // calc lidar scan end time
      lidar_pushed = true;
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default: {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  std::cerr << "out sync" << std::endl;
}

void LIVMapper::publish_frame_world(VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/all_pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/all_image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}
