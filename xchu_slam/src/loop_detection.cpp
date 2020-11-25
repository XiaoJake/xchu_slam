/**
* @Program: Project
* @Description: [用一句话描述此类]
* @Author: Xiangcheng Hu
* @Create: 2020/11/25
* @Copyright: [2020] <Copyright hxc@2022087641@qq.com>
**/


#include "xchu_slam/loop_detection.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "loop_node");

  ros::NodeHandle nh, pnh("~");
  LoopDetection *map = new LoopDetection(nh, pnh);

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    map->run();
    rate.sleep();
  }

  return 0;
}

LoopDetection::LoopDetection(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh) {
  // 初始化gtsam参数
  if (!init()) {
    exit(-1);
  }

  pub_updated_pose_ = nh_.advertise<nav_msgs::Odometry>("/final_odom", 1); //  优化之后的pose

  //  sub_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/top/rslidar_points", 5, boost::bind(&XCHUSlam::pcCB, this, _1));
  sub_pc_ =
      nh_.subscribe<sensor_msgs::PointCloud2>(_lidar_topic.c_str(), 100, boost::bind(&LoopDetection::pcCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/updated_odom", 10, boost::bind(&LoopDetection::odomCB, this, _1));

}

bool LoopDetection::init() {
  // 初始化
  pnh_.param<float>("scan_period", scan_period_, 0.1);
  pnh_.param<float>("keyframe_dist", keyframe_dist_, 0.5);
  // pnh_.param<float>("surround_search_radius", surround_search_radius_, 20);
  pnh_.param<int>("surround_search_num", surround_search_num_, 10);
  pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.5);
  pnh_.param<float>("min_scan_range", min_scan_range_, 2);
  min_scan_range_ *= min_scan_range_;
  pnh_.param<float>("max_scan_range", max_scan_range_, 120);
  max_scan_range_ *= max_scan_range_;
  pnh_.param<bool>("use_odom", use_odom_, false);
  pnh_.param<bool>("use_imu", use_imu_, false);
  pnh_.param<bool>("loop_closure_enabled", loop_closure_enabled_, true);

  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size, 0.1);
  pnh_.param<double>("ndt_res", ndt_res_, 2.0);
  pnh_.param<int>("max_iters", max_iters_, 50);

  pnh_.param<float>("history_search_radius", history_search_radius_, 10);
  pnh_.param<int>("history_search_num", history_search_num_, 10);
  pnh_.param<float>("history_fitness_score", history_fitness_score_, 0.3);
  pnh_.param<float>("ds_history_size", ds_history_size_, 1);

  pnh_.param<std::string>("save_dir", save_dir_, "");

  pnh_.param<std::string>("imu_topic", _imu_topic, "/imu_raw");
  pnh_.param<std::string>("odom_topic", _odom_topic, "/odom_raw");
  pnh_.param<std::string>("lidar_topic", _lidar_topic, "/velodyne_points");

  std::cout << "imu topic: " << _imu_topic << std::endl;
  std::cout << "odom topic: " << _odom_topic << std::endl;
  std::cout << "lidar topic: " << _lidar_topic << std::endl;
  std::cout << std::endl;

  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  // ndt参数
  //    cpu_ndt_.setTransformationEpsilon(trans_eps_);
  //    cpu_ndt_.setResolution(ndt_res_);
  //    cpu_ndt_.setStepSize(step_size);
  //    cpu_ndt_.setMaximumIterations(max_iters_);
  ndt->setTransformationEpsilon(trans_eps_);
  ndt->setResolution(ndt_res_);
  ndt->setStepSize(step_size);
  ndt->setMaximumIterations(max_iters_);
  ndt_omp_ = ndt;
  /* cpu_ndt_.setTransformationEpsilon(trans_eps_);
   cpu_ndt_.setStepSize(step_size);
   cpu_ndt_.setResolution(ndt_res_);
   cpu_ndt_.setMaximumIterations(max_iters_);*/

  // 初始化tf
  tf_b2l_ = Eigen::Matrix4f::Identity();
  float roll, pitch, yaw;
  if (!nh_.getParam("tf_b2l_x", tf_b2l_(0, 3)) || !nh_.getParam("tf_b2l_y", tf_b2l_(1, 3)) ||
      !nh_.getParam("tf_b2l_z", tf_b2l_(2, 3)) || !nh_.getParam("tf_b2l_roll", roll) ||
      !nh_.getParam("tf_b2l_pitch", pitch) || !nh_.getParam("tf_b2l_yaw", yaw)) {
    ROS_ERROR("transform between /base_link to /laser not set.");
    exit(-1);
  }
  Eigen::AngleAxisf rx(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf ry(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rz(yaw, Eigen::Vector3f::UnitZ());
  tf_b2l_.block(0, 0, 3, 3) = (rz * ry * rx).matrix();

  // 不同的下采样网格大小
  downSizeFilterGlobalMapKeyFrames.setLeafSize(0.5, 0.5, 0.5); // for global map visualization
  downSizeFilterLocalmap.setLeafSize(1.0, 1.0, 1.0);// 发布localmap
  downSizeFilterGlobalMap.setLeafSize(1.0, 1.0, 1.0); // 保存全局地图时下采样

  voxel_filter_.setLeafSize(0.5, 0.5, 0.5);
  ds_source_.setLeafSize(0.5, 0.5, 0.5);
  ds_history_keyframes_.setLeafSize(0.5, 0.5, 0.5); // 回环检测时回环历史帧的localmap下采样大小

  // gtsam参数初始化
  ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam = new ISAM2(params);
  gtsam::Vector vector6(6);
  vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  prior_noise_ = noiseModel::Diagonal::Variances(vector6);
  odom_noise_ = noiseModel::Diagonal::Variances(vector6);

  imu_ptr_front_ = odom_ptr_front_ = 0;
  imu_ptr_last_ = odom_ptr_last_ = -1;
  imu_ptr_last_iter_ = odom_ptr_last_iter_ = 0;

  pre_pose_m_ = cur_pose_m_ = pre_pose_o_ = cur_pose_o_ = Eigen::Matrix4f::Identity();

  tf_m2o_.setIdentity();

  loop_closed_ = false;

  // 地面去除的设定
  ground_filter.setIfClipHeight(false);
  ground_filter.setMinDistance(1.0);

  // 初始化点云相关变量
  pc_source_.reset(new pcl::PointCloud<PointT>());
  pc_target_.reset(new pcl::PointCloud<PointT>());

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointXYZIRPYT>());

  latest_keyframe_.reset(new pcl::PointCloud<PointT>());
  near_history_keyframes_.reset(new pcl::PointCloud<PointT>());

  kdtree_poses_.reset(new pcl::KdTreeFLANN<PointT>());

  imu_time_.fill(0);
  imu_roll_.fill(0);
  imu_pitch_.fill(0);
  imu_yaw_.fill(0);

  imu_acc_x_.fill(0);
  imu_acc_y_.fill(0);
  imu_acc_z_.fill(0);
  imu_velo_x_.fill(0);
  imu_velo_y_.fill(0);
  imu_velo_z_.fill(0);
  imu_shift_x_.fill(0);
  imu_shift_y_.fill(0);
  imu_shift_z_.fill(0);

  imu_angular_velo_x_.fill(0);
  imu_angular_velo_y_.fill(0);
  imu_angular_velo_z_.fill(0);
  imu_angular_rot_x_.fill(0);
  imu_angular_rot_y_.fill(0);
  imu_angular_rot_z_.fill(0);

  ROS_INFO("init params.");
  return true;
}

void LoopDetection::run() {
  std::cout << "cloud buffer size : " << cloudBuf.size() << std::endl;

  if (!cloudBuf.empty() && !odomBuf.empty()) {
    //align time stamp
    mutex_lock.lock();
    if (!odomBuf.empty() && odomBuf.front()->header.stamp.toSec()
        < cloudBuf.front()->header.stamp.toSec() - 0.5 * 0.1) {
      ROS_WARN(
          "loop_detection: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",
          odomBuf.front()->header.stamp.toSec(),
          cloudBuf.front()->header.stamp.toSec());
      odomBuf.pop();
      mutex_lock.unlock();
      return;
    }

    if (!cloudBuf.empty() && cloudBuf.front()->header.stamp.toSec()
        < odomBuf.front()->header.stamp.toSec() - 0.5 * 0.1) {
      ROS_WARN(
          "loop_detection: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",
          odomBuf.front()->header.stamp.toSec(),
          cloudBuf.front()->header.stamp.toSec());
      cloudBuf.pop();
      mutex_lock.unlock();
      return;
    }

    pcl::PointCloud<PointT>::Ptr pointcloud_in(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(cloudBuf.front(), *pointcloud_in); // 最新的点云帧
    ros::Time pointcloud_time = (cloudBuf.front())->header.stamp;
    Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity(); // 最新的odom
    odom_in.rotate(Eigen::Quaterniond(odomBuf.front()->pose.pose.orientation.w,
                                      odomBuf.front()->pose.pose.orientation.x,
                                      odomBuf.front()->pose.pose.orientation.y,
                                      odomBuf.front()->pose.pose.orientation.z));
    odom_in.pretranslate(Eigen::Vector3d(odomBuf.front()->pose.pose.position.x,
                                         odomBuf.front()->pose.pose.position.y,
                                         odomBuf.front()->pose.pose.position.z));
    odomBuf.pop();
    cloudBuf.pop();
    mutex_lock.unlock();

    // 进行回环检测
    saveKeyframesAndFactor();

    // 回环更新pose
    correctPoses();



    performLoopClosure();


    // 发布isc图像
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id = "velodyne";
    out_msg.header.stamp = pointcloud_time;
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = iscGeneration.getLastISCRGB();
    isc_pub.publish(out_msg.toImageMsg());

    // 发布回环检测的结果
    iscloam::LoopInfo loop;
    loop.header.stamp = pointcloud_time;
    loop.header.frame_id = "velodyne";
    loop.current_id = iscGeneration.current_frame_id;
    for (int i = 0; i < (int) iscGeneration.matched_frame_id.size(); i++) {
      loop.matched_id.push_back(iscGeneration.matched_frame_id[i]);
    }
    loop_info_pub.publish(loop);
  }
  //sleep 2 ms every time

}

void LoopDetection::visualThread() {
  ros::Rate rate(0.2);
  while (ros::ok()) {
    rate.sleep();
    publishKeyposesAndFrames();
  }

  // 关闭终端时保存地图
  TransformAndSaveFinalMap();
}

/**
 * 保存全局地图
 *
 */
void LoopDetection::TransformAndSaveFinalMap() {
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr finalmap_ptr(new pcl::PointCloud<PointT>());
  // pcl::PointCloud<PointT>::Ptr map_no_ground(new pcl::PointCloud<PointT>());

  for (int i = 0; i < cloud_keyframes_.size(); ++i) {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
    *finalmap_ptr += *tmp;
    num_points += tmp->points.size();

    // 保存去除地面的点云
    //    pcl::PointCloud<PointT>::Ptr tmp1(new pcl::PointCloud<PointT>());
    //    ground_filter.convert(cloud_keyframes_[i], tmp1);
    //    tmp1 = transformPointCloud(tmp1, cloud_keyposes_6d_->points[i]);
    //    *map_no_ground += *tmp1;
    //    num_points1 += tmp1->points.size();
  }

  finalmap_ptr->width = finalmap_ptr->points.size();
  finalmap_ptr->height = 1;
  finalmap_ptr->is_dense = false;

  // 保存原图的话就不需要下采样了
  downSizeFilterGlobalMap.setInputCloud(finalmap_ptr);
  downSizeFilterGlobalMap.filter(*finalmap_ptr);

  //  map_no_ground->width = map_no_ground->points.size();
  //  map_no_ground->height = 1;
  //  map_no_ground->is_dense = false;

  // 优化后的位姿也保存一份
  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(save_dir_ + "trajectory_" + stamp + ".pcd", *poses);
  pcl::io::savePCDFile(save_dir_ + "finalCloud_" + stamp + ".pcd", *finalmap_ptr);
  // pcl::io::savePCDFile(save_dir_ + "final_no_ground_" + stamp + ".pcd", *map_no_ground);

  ROS_WARN("Save map. pose size: %d, cloud size: %d", poses->points.size(), num_points);

  //    Eigen::Translation3f tf_t(init_tf_x, init_tf_y, init_tf_z);         // tl: translation
  //    Eigen::AngleAxisf rot_x(init_tf_roll, Eigen::Vector3f::UnitX());    // rot: rotation
  //    Eigen::AngleAxisf rot_y(init_tf_pitch, Eigen::Vector3f::UnitY());
  //    Eigen::AngleAxisf rot_z(init_tf_yaw, Eigen::Vector3f::UnitZ());
  //    Eigen::Matrix4f map_to_init_trans_matrix = (tf_t * rot_z * rot_y * rot_x).matrix();
  //
  //    pcl::PointCloud<PointType>::Ptr transformed_pc_ptr(new pcl::PointCloud<PointType>());
  //
  //    pcl::transformPointCloud(*globalMapKeyFramesDS, *transformed_pc_ptr, map_to_init_trans_matrix);
  //
  //    pcl::io::savePCDFileASCII(pcd_file_path + "finalCloud.pcd", *transformed_pc_ptr);
  //
  //    // 后面的各种特征点地图可以注释掉
  //    pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
  //    pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
  //    pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
  //    pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());
  //
  //    for (int i = 0; i < cornerCloudKeyFrames.size(); i++) {
  //        *cornerMapCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
  //        *surfaceMapCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
  //        *surfaceMapCloud += *transformPointCloud(outlierCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
  //    }
  //    downSizeFilterCorner.setInputCloud(cornerMapCloud);
  //    downSizeFilterCorner.filter(*cornerMapCloudDS);
  //    downSizeFilterSurf.setInputCloud(surfaceMapCloud);
  //    downSizeFilterSurf.filter(*surfaceMapCloudDS);
  //
  //    pcl::transformPointCloud(*globalMapKeyFramesDS, *transformed_pc_ptr, map_to_init_trans_matrix);
  //    pcl::io::savePCDFileASCII(pcd_file_path + "cornerMap.pcd", *cornerMapCloudDS);
  //    pcl::io::savePCDFileASCII(pcd_file_path + "surfaceMap.pcd", *surfaceMapCloudDS);
  //    pcl::io::savePCDFileASCII(pcd_file_path + "trajectory.pcd", *cloudKeyPoses3D);
}

/**
 * @brief 参考 loam 的点云去运动畸变（基于匀速运动假设）
 *
 */
void LoopDetection::adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time) {
  bool half_passed = false;
  int cloud_size = cloud->points.size();

  //  计算当前帧雷达转过的角度
  float start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
  float end_ori = -std::atan2(cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
  if (end_ori - start_ori > 3 * M_PI) {
    end_ori -= 2 * M_PI;
  } else if (end_ori - start_ori < M_PI) {
    end_ori += 2 * M_PI;
  }
  float ori_diff = end_ori - start_ori;

  Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
  Eigen::Vector3f shift_from_start;
  Eigen::Matrix3f r_s_i, r_c;
  Eigen::Vector3f adjusted_p;
  float ori_h;
  for (int i = 0; i < cloud_size; ++i) {
    PointT &p = cloud->points[i];
    ori_h = -std::atan2(p.y, p.x);
    if (!half_passed) {
      if (ori_h < start_ori - M_PI * 0.5) {
        ori_h += 2 * M_PI;
      } else if (ori_h > start_ori + M_PI * 1.5) {
        ori_h -= 2 * M_PI;
      }

      if (ori_h - start_ori > M_PI) {
        half_passed = true;
      }
    } else {
      ori_h += 2 * M_PI;
      if (ori_h < end_ori - 1.5 * M_PI) {
        ori_h += 2 * M_PI;
      } else if (ori_h > end_ori + 0.5 * M_PI) {
        ori_h -= 2 * M_PI;
      }
    }

    float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

    if (imu_ptr_last_ > 0) {
      imu_ptr_front_ = imu_ptr_last_iter_;
      while (imu_ptr_front_ != imu_ptr_last_) {
        if (scan_time + rel_time < imu_time_[imu_ptr_front_]) {
          break;
        }
        imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
      }
      if (std::abs(scan_time + rel_time - imu_time_[imu_ptr_front_]) > scan_period_) {
        ROS_WARN_COND(i < 10, "unsync imu and pc msg");
        continue;
      }

      if (scan_time + rel_time > imu_time_[imu_ptr_front_]) {
        rpy_cur(0) = imu_roll_[imu_ptr_front_];
        rpy_cur(1) = imu_pitch_[imu_ptr_front_];
        rpy_cur(2) = imu_yaw_[imu_ptr_front_];
        shift_cur(0) = imu_shift_x_[imu_ptr_front_];
        shift_cur(1) = imu_shift_y_[imu_ptr_front_];
        shift_cur(2) = imu_shift_z_[imu_ptr_front_];
        velo_cur(0) = imu_velo_x_[imu_ptr_front_];
        velo_cur(1) = imu_velo_y_[imu_ptr_front_];
        velo_cur(2) = imu_velo_z_[imu_ptr_front_];
      } else {
        int imu_ptr_back = (imu_ptr_front_ - 1 + imu_queue_len_) % imu_queue_len_;
        float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) /
            (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
        float ratio_back = 1. - ratio_front;
        rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] * ratio_back;
        rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] * ratio_back;
        rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
        shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] * ratio_back;
        shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] * ratio_back;
        shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] * ratio_back;
        velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] * ratio_back;
        velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] * ratio_back;
        velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] * ratio_back;
      }

      r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

      if (i == 0) {
        rpy_start = rpy_cur;
        shift_start = shift_cur;
        velo_start = velo_cur;
        r_s_i = r_c.inverse();
      } else {
        shift_from_start = shift_cur - shift_start - velo_start * rel_time;
        adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
        p.x = adjusted_p.x();
        p.y = adjusted_p.y();
        p.z = adjusted_p.z();
      }
    }
    imu_ptr_last_iter_ = imu_ptr_front_;
  }

  // 发布去完畸变的点云帧
  if (pub_undistorted_pc_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp.fromSec(scan_time);
    msg.header.frame_id = "/laser";
    pub_undistorted_pc_.publish(msg);
  }
}

/**
 * @brief 提取附近点云作为 target map。
 * 在 loop_closure_enabled_ = false 的情况下目前不需要提取，因为可以通过 updateVoxelGrid 更新 target map
 *
 */
void LoopDetection::extractLocalmap() {
  // 没有关键帧
  if (cloud_keyframes_.empty()) {
    ROS_WARN("No keyFrames...");
    return;
  }

  bool target_updated = false; // 是否更新
  if (loop_closure_enabled_) {
    // 关键帧数量不够，加进来
    if (recent_keyframes_.size() < surround_search_num_) {
      recent_keyframes_.clear();
      // cloud_keyposes_3d_是当前的点云帧
      for (int i = cloud_keyposes_3d_->points.size() - 1; i >= 0; --i) {
        int this_key_id = int(cloud_keyposes_3d_->points[i].intensity);
        // 加进去的每一帧都做好了转换
        pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
        tf_cloud = transformPointCloud(cloud_keyframes_[this_key_id], cloud_keyposes_6d_->points[this_key_id]);
        recent_keyframes_.push_back(tf_cloud);
        if (recent_keyframes_.size() >= surround_search_num_) {
          break;
        }
      }
      target_updated = true;
    } else {
      // localmap里面帧数够了，把最老的帧pop出来
      static int latest_frame_id = cloud_keyframes_.size() - 1;
      if (latest_frame_id != cloud_keyframes_.size() - 1) {
        latest_frame_id = cloud_keyframes_.size() - 1;
        recent_keyframes_.pop_back();
        pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
        tf_cloud = transformPointCloud(cloud_keyframes_[latest_frame_id], cloud_keyposes_6d_->points[latest_frame_id]);
        recent_keyframes_.push_front(tf_cloud);
        target_updated = true;
      }
    }
  }

  // 将附近的点云帧作为localmap匹配的target，localmap变化的时候需要更新一下target_cloud
  if (target_updated) {
    pc_target_->clear();
    for (auto keyframe : recent_keyframes_) {
      *pc_target_ += *keyframe;
    }
    // cpu_ndt_.setInputTarget(pc_target_);
    ndt_omp_->setInputTarget(pc_target_);
  }

  // 发布实时的localmap
  if (pub_localmap_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*pc_target_, *target_cloud);
    downSizeFilterLocalmap.setInputCloud(target_cloud);
    downSizeFilterLocalmap.filter(*target_cloud);  // 下采样
    pcl::toROSMsg(*target_cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub_localmap_.publish(msg);
  }
}

/**
 * @brief 保存关键帧及其对应位姿，更新位姿图
 * 移动距离作为关键帧选取标准
 *
 */
bool LoopDetection::saveKeyframesAndFactor() {
  // 此处的当前位姿(cur_pose_ndt_)为ndt匹配后的final_transformation
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y,
                               cur_pose_ndt_.pose.pose.orientation.z, cur_pose_ndt_.pose.pose.orientation.w)).getRPY(
      roll, pitch, yaw);

  // 第一帧进来的时候直接添加PriorFactor
  if (cloud_keyposes_3d_->points.empty()) {
    // gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(roll, pitch, yaw),
                                                Point3(cur_pose_ndt_.pose.pose.position.x,
                                                       cur_pose_ndt_.pose.pose.position.y,
                                                       cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    initial_estimate_.insert(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w,
                                                       cur_pose_ndt_.pose.pose.orientation.x,
                                                       cur_pose_ndt_.pose.pose.orientation.y,
                                                       cur_pose_ndt_.pose.pose.orientation.z),
                                      Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y,
                                             cur_pose_ndt_.pose.pose.position.z)));
    // 更新current_pose
    pre_keypose_ = cur_pose_ndt_;
  } else {
    // 关集帧之间的距离大于0.5米才添加进来
    const auto &pre_pose = cloud_keyposes_6d_->points[cloud_keyposes_3d_->points.size() - 1];
    if (std::pow(cur_pose_ndt_.pose.pose.position.x - pre_pose.x, 2) +
        std::pow(cur_pose_ndt_.pose.pose.position.y - pre_pose.y, 2) +
        std::pow(cur_pose_ndt_.pose.pose.position.z - pre_pose.z, 2) <
        keyframe_dist_ * keyframe_dist_) {

      ROS_WARN("too close...");
      return false;
    }

    // 添加相邻帧之间的约束关系 BetweenFactor
    gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(pre_pose.roll, pre_pose.pitch, pre_pose.yaw),
                                   Point3(pre_pose.x, pre_pose.y, pre_pose.z));
    gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw),
                                 Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y,
                                        cur_pose_ndt_.pose.pose.position.z * 0));
    gtSAMgraph_.add(BetweenFactor<Pose3>(cloud_keyposes_3d_->points.size() - 1, cloud_keyposes_3d_->points.size(),
                                         pose_from.between(pose_to), odom_noise_));
    initial_estimate_.insert(cloud_keyposes_3d_->points.size(), Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw),
                                                                      Point3(cur_pose_ndt_.pose.pose.position.x,
                                                                             cur_pose_ndt_.pose.pose.position.y,
                                                                             cur_pose_ndt_.pose.pose.position.z * 0)));

  }

  isam->update(gtSAMgraph_, initial_estimate_);
  isam->update();

  gtSAMgraph_.resize(0);
  initial_estimate_.clear();

  PointT this_pose_3d;
  PointXYZIRPYT this_pose_6d;
  Pose3 latest_estimate;
  isam_current_estimate_ = isam->calculateEstimate();
  latest_estimate = isam_current_estimate_.at<Pose3>(isam_current_estimate_.size() - 1);

  // intensity字段表示当前关键帧的序号
  this_pose_6d.x = this_pose_3d.x = latest_estimate.translation().x();
  this_pose_6d.y = this_pose_3d.y = latest_estimate.translation().y();
  this_pose_6d.z = this_pose_3d.z = latest_estimate.translation().z();
  this_pose_6d.intensity = this_pose_3d.intensity = cloud_keyposes_3d_->points.size();
  this_pose_6d.roll = latest_estimate.rotation().roll();
  this_pose_6d.pitch = latest_estimate.rotation().pitch();
  this_pose_6d.yaw = latest_estimate.rotation().yaw();
  this_pose_6d.time = cur_pose_ndt_.header.stamp.toSec();

  // 添加关键帧
  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyposes_6d_->points.push_back(this_pose_6d);

  if (cloud_keyposes_3d_->points.size() > 1) {
    pre_keypose_.pose.pose.position.x = this_pose_3d.x;
    pre_keypose_.pose.pose.position.y = this_pose_3d.y;
    pre_keypose_.pose.pose.position.z = this_pose_3d.z;
    pre_keypose_.pose.pose.orientation.w = latest_estimate.rotation().toQuaternion().w();
    pre_keypose_.pose.pose.orientation.x = latest_estimate.rotation().toQuaternion().x();
    pre_keypose_.pose.pose.orientation.y = latest_estimate.rotation().toQuaternion().y();
    pre_keypose_.pose.pose.orientation.z = latest_estimate.rotation().toQuaternion().z();
    pre_keypose_.header.stamp = cur_pose_ndt_.header.stamp;
  }

  // 更新当前pose
  cur_pose_m_.block<3, 3>(0, 0) = Eigen::Quaternionf(pre_keypose_.pose.pose.orientation.w,
                                                     pre_keypose_.pose.pose.orientation.x,
                                                     pre_keypose_.pose.pose.orientation.y,
                                                     pre_keypose_.pose.pose.orientation.z).toRotationMatrix();
  cur_pose_m_(0, 3) = pre_keypose_.pose.pose.position.x;
  cur_pose_m_(1, 3) = pre_keypose_.pose.pose.position.y;
  cur_pose_m_(2, 3) = pre_keypose_.pose.pose.position.z;

  pcl::PointCloud<PointT>::Ptr cur_keyframe(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*pc_source_, *cur_keyframe);
  for (auto &p : cur_keyframe->points) {
    p.intensity = this_pose_3d.intensity;
  }
  cloud_keyframes_.push_back(cur_keyframe);

  // 更新pc_source
  pcl::PointCloud<PointT>::Ptr pc_m(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*pc_source_, *pc_m, cur_pose_m_);
  // cpu_ndt_.updateVoxelGrid(pc_m);

  // 更新位姿
  pre_pose_m_ = cur_pose_m_;
  pre_pose_ndt_ = cur_pose_ndt_;

  ROS_INFO("saveKeyframesAndFactor: %d points", cur_keyframe->points.size());
  return true;
}

void LoopDetection::publishKeyposesAndFrames() {
  // 发布关键帧位姿
  if (pub_keyposes_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_keyposes_.publish(msg);
  }
  // 发布全局地图，非最终地图，只是里程计关键帧拼接而成的
  if (pub_globalmap_.getNumSubscribers() > 0) {
    int num_points = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<PointT>::Ptr globalmap_ptr(new pcl::PointCloud<PointT>());
    Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
    // 把关键帧拼接而成，这里关键帧及其对应的位姿都存储好了
    for (int i = 0; i < cloud_keyframes_.size(); ++i) {
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
      tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
      *globalmap_ptr += *tmp;
      num_points += tmp->points.size();
    }

    // downsample visualized points
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalmap_ptr);
    downSizeFilterGlobalMapKeyFrames.filter(*globalmap_ptr);

    pcl::toROSMsg(*globalmap_ptr, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_globalmap_.publish(msg);
    ROS_INFO("Map Size: %d points", num_points);
  }
}

void LoopDetection::process_cloud(const pcl::PointCloud<PointT> &tmp, const ros::Time &current_scan_time) {
  auto start = std::chrono::system_clock::now();
  // 提取附近的点云帧，用一个栈来实时维护localamp
  extractLocalmap();

  // 点云转换到地图坐标系中处理
  pc_source_->clear();
  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>(tmp));
  pcl::transformPointCloud(*tmp_cloud, *pc_source_, tf_b2l_);

  // 去运动畸变
  if (use_imu_) {
    adjustDistortion(pc_source_, current_scan_time.toSec());
  }

  tmp_cloud->clear();
  voxel_filter_.setInputCloud(pc_source_);
  voxel_filter_.filter(*tmp_cloud);

  pc_source_->clear(); // 去除较近较远的点云继续存在pc_source_中
  float r;
  for (const auto &p : tmp_cloud->points) {
    r = p.x * p.x + p.y * p.y;
    if (r > min_scan_range_ && r < max_scan_range_) {
      pc_source_->points.push_back(p);
    }
  }

  // 第一帧点云进来, 直接存为target, 初始化起始位置
  if (cloud_keyframes_.empty()) {
    ROS_INFO("first laser frame.");
    *pc_target_ += *pc_source_;
    //    cpu_ndt_.setInputTarget(pc_target_);
    ndt_omp_->setInputTarget(pc_target_);

    // 使用里程计
    if (use_odom_ && odom_ptr_last_ != -1) {
      int odom_ptr = odom_ptr_front_;
      while (odom_ptr != odom_ptr_last_) {
        if (odom_queue_[odom_ptr].header.stamp > current_scan_time) {
          break;
        }
        odom_ptr = (odom_ptr + 1) % imu_queue_len_;
      }
      pre_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.x,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.y,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
      pre_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
      pre_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
      pre_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;
      cur_pose_o_ = pre_pose_o_;
      odom_ptr_front_ = odom_ptr; // 更新指针
    }
  }

  // 配准初始位姿估计, 使用编码器做初值
  nav_msgs::Odometry predict_msg;
  if (use_odom_ && odom_ptr_last_ != -1) {
    pre_pose_o_ = cur_pose_o_;
    int odom_ptr = odom_ptr_front_;
    while (odom_ptr != odom_ptr_last_) {
      if (odom_queue_[odom_ptr].header.stamp > current_scan_time) {
        break;
      }
      odom_ptr = (odom_ptr + 1) % imu_queue_len_;
    }
    cur_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.x,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.y,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
    cur_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
    cur_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
    cur_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;
    odom_ptr_front_ = odom_ptr; // 更新指针

    Eigen::Quaternionf tmp_q(pre_pose_m_.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
    pre_pose_m_.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Matrix4f r_m2o = Eigen::Matrix4f::Identity();
    r_m2o.block<3, 3>(0, 0) = Eigen::Quaternionf(tf_m2o_.getRotation().w(), tf_m2o_.getRotation().x(),
                                                 tf_m2o_.getRotation().y(),
                                                 tf_m2o_.getRotation().z()).toRotationMatrix();
    cur_pose_m_ = pre_pose_m_ * pre_pose_o_.inverse() * cur_pose_o_; // 见笔记“预测当前位姿”

    tmp_q = Eigen::Quaternionf(cur_pose_m_.block<3, 3>(0, 0));
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

    predict_msg.header.stamp = current_scan_time;
    predict_msg.header.frame_id = "map";
    predict_msg.pose.pose.position.x = cur_pose_m_(0, 3);
    predict_msg.pose.pose.position.y = cur_pose_m_(1, 3);
    predict_msg.pose.pose.position.z = cur_pose_m_(2, 3);
    predict_msg.pose.pose.orientation.w = tmp_q.w();
    predict_msg.pose.pose.orientation.x = tmp_q.x();
    predict_msg.pose.pose.orientation.y = tmp_q.y();
    predict_msg.pose.pose.orientation.z = tmp_q.z();
    pub_predict_pose_.publish(predict_msg);
  } else {
    // TODO: 改进
    predict_pose_ = pre_pose_ndt_;
    cur_pose_m_ = pre_pose_m_;
  }

  // ndt 匹配迭代
  //  cpu_ndt_.setInputSource(pc_source_);
  //  cpu_ndt_.align(cur_pose_m_);
  //  fitness_score_ = cpu_ndt_.getFitnessScore();
  //  final_transformation_ = cpu_ndt_.getFinalTransformation();
  //  has_converged_ = cpu_ndt_.hasConverged();
  //  final_iters_ = cpu_ndt_.getFinalNumIteration();

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud_(new pcl::PointCloud<pcl::PointXYZI>);
  ndt_omp_->setInputSource(pc_source_);
  ndt_omp_->align(*aligned_cloud_, cur_pose_m_);
  fitness_score_ = ndt_omp_->getFitnessScore();
  final_transformation_ = ndt_omp_->getFinalTransformation();
  has_converged_ = ndt_omp_->hasConverged();
  final_iters_ = ndt_omp_->getFinalNumIteration();

  Eigen::Quaternionf tmp_q(final_transformation_.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  cur_pose_ndt_.pose.pose.position.x = final_transformation_(0, 3);
  cur_pose_ndt_.pose.pose.position.y = final_transformation_(1, 3);
  cur_pose_ndt_.pose.pose.position.z = final_transformation_(2, 3);
  cur_pose_ndt_.pose.pose.orientation.w = tmp_q.w();
  cur_pose_ndt_.pose.pose.orientation.x = tmp_q.x();
  cur_pose_ndt_.pose.pose.orientation.y = tmp_q.y();
  cur_pose_ndt_.pose.pose.orientation.z = tmp_q.z();
  cur_pose_ndt_.header.stamp = current_scan_time;
  cur_pose_m_ = final_transformation_;

  // 发布里程计位姿
  nav_msgs::Odometry updated_msg;
  updated_msg.header.stamp = current_scan_time;
  updated_msg.header.frame_id = "map";
  updated_msg.pose = cur_pose_ndt_.pose;
  pub_updated_pose_.publish(updated_msg);

  tf::Transform tf_m2b;
  tf_m2b.setOrigin(
      tf::Vector3(final_transformation_(0, 3), final_transformation_(1, 3), final_transformation_(2, 3)));
  tf_m2b.setRotation(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w()));

  if (use_odom_ && odom_ptr_last_ != -1) {
    tf_o2b_.setOrigin(tf::Vector3(odom_queue_[odom_ptr_front_].pose.pose.position.x,
                                  odom_queue_[odom_ptr_front_].pose.pose.position.y,
                                  odom_queue_[odom_ptr_front_].pose.pose.position.z));
    tf_o2b_.setRotation(tf::Quaternion(odom_queue_[odom_ptr_front_].pose.pose.orientation.x,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.y,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.z,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.w));
    tf_m2o_ = tf_m2b * tf_o2b_.inverse();
    // tf::StampedTransform tmp;
    // tf_listener_.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.05));
    // tf_listener_.lookupTransform("/odom", "/base_link", ros::Time(0), tmp);
    // tf_m2o_ = tf_m2b * tmp.inverse();
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2o_, current_scan_time, "map", "/odom"));
  } else {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2b, current_scan_time, "map", "/base_link"));
  }

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "current cloud size: " << pc_source_->size() << " points." << std::endl;
  std::cout << "target cloud size: " << pc_target_->points.size() << " points." << std::endl;
  std::cout << "iteration: " << final_iters_ << std::endl;
  std::cout << "fitness score: " << fitness_score_ << std::endl;
  std::cout << "transformation matrix:" << std::endl;
  std::cout << final_transformation_ << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(final_transformation_(0, 3) - pre_pose_m_(0, 3), 2) +
      std::pow(final_transformation_(1, 3) - pre_pose_m_(1, 3), 2) +
      std::pow(final_transformation_(2, 3) - pre_pose_m_(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void LoopDetection::odomCB(const nav_msgs::OdometryConstPtr &msg) {
  mutex_lock.lock();
  odomBuf.push(*msg);
  mutex_lock.unlock();
}

void LoopDetection::pcCB(const sensor_msgs::PointCloud2ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  pcl::PointCloud<PointT> tmp;
  pcl::fromROSMsg(*msg, tmp);
  std::vector<int> indices;   //remove NAN
  pcl::removeNaNFromPointCloud(tmp, tmp, indices);

  // 点云匹配
  process_cloud(tmp, msg->header.stamp);

  // 保存位姿
  saveKeyframesAndFactor();

  // 回环更新pose
  correctPoses();

  /*// 提取附近的点云帧，用一个栈来实时维护localamp
  extractLocalmap();

  // 点云转换到地图坐标系中处理
  pc_source_->clear();
  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*msg, *tmp_cloud);
  pcl::transformPointCloud(*tmp_cloud, *pc_source_, tf_b2l_);

  // 去运动畸变
  if (use_imu_) {
    adjustDistortion(pc_source_, msg->header.stamp.toSec());
  }

  tmp_cloud->clear();
  voxel_filter_.setInputCloud(pc_source_);
  voxel_filter_.filter(*tmp_cloud);

  pc_source_->clear(); // 去除较近较远的点云继续存在pc_source_中
  float r;
  for (const auto &p : tmp_cloud->points) {
    r = p.x * p.x + p.y * p.y;
    if (r > min_scan_range_ && r < max_scan_range_) {
      pc_source_->points.push_back(p);
    }
  }

  // 第一帧点云进来, 直接存为target, 初始化起始位置
  if (cloud_keyframes_.empty()) {
    ROS_INFO("first laser frame.");
    *pc_target_ += *pc_source_;
    //    cpu_ndt_.setInputTarget(pc_target_);
    ndt_omp_->setInputTarget(pc_target_);

    // 使用里程计
    if (use_odom_ && odom_ptr_last_ != -1) {
      int odom_ptr = odom_ptr_front_;
      while (odom_ptr != odom_ptr_last_) {
        if (odom_queue_[odom_ptr].header.stamp > msg->header.stamp) {
          break;
        }
        odom_ptr = (odom_ptr + 1) % imu_queue_len_;
      }
      pre_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.x,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.y,
                                                         odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
      pre_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
      pre_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
      pre_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;
      cur_pose_o_ = pre_pose_o_;
      odom_ptr_front_ = odom_ptr; // 更新指针
    }
  }

  // 配准初始位姿估计, 使用编码器做初值
  nav_msgs::Odometry predict_msg;
  if (use_odom_ && odom_ptr_last_ != -1) {
    pre_pose_o_ = cur_pose_o_;
    int odom_ptr = odom_ptr_front_;
    while (odom_ptr != odom_ptr_last_) {
      if (odom_queue_[odom_ptr].header.stamp > msg->header.stamp) {
        break;
      }
      odom_ptr = (odom_ptr + 1) % imu_queue_len_;
    }
    cur_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.x,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.y,
                                                       odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
    cur_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
    cur_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
    cur_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;
    odom_ptr_front_ = odom_ptr; // 更新指针

    Eigen::Quaternionf tmp_q(pre_pose_m_.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
    pre_pose_m_.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Matrix4f r_m2o = Eigen::Matrix4f::Identity();
    r_m2o.block<3, 3>(0, 0) = Eigen::Quaternionf(tf_m2o_.getRotation().w(), tf_m2o_.getRotation().x(),
                                                 tf_m2o_.getRotation().y(),
                                                 tf_m2o_.getRotation().z()).toRotationMatrix();
    cur_pose_m_ = pre_pose_m_ * pre_pose_o_.inverse() * cur_pose_o_; // 见笔记“预测当前位姿”

    tmp_q = Eigen::Quaternionf(cur_pose_m_.block<3, 3>(0, 0));
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

    predict_msg.header.stamp = msg->header.stamp;
    predict_msg.header.frame_id = "map";
    predict_msg.pose.pose.position.x = cur_pose_m_(0, 3);
    predict_msg.pose.pose.position.y = cur_pose_m_(1, 3);
    predict_msg.pose.pose.position.z = cur_pose_m_(2, 3);
    predict_msg.pose.pose.orientation.w = tmp_q.w();
    predict_msg.pose.pose.orientation.x = tmp_q.x();
    predict_msg.pose.pose.orientation.y = tmp_q.y();
    predict_msg.pose.pose.orientation.z = tmp_q.z();
    pub_predict_pose_.publish(predict_msg);
  } else {
    // TODO: 改进
    predict_pose_ = pre_pose_ndt_;
    cur_pose_m_ = pre_pose_m_;
  }

  // ndt 匹配迭代
  //  cpu_ndt_.setInputSource(pc_source_);
  //  cpu_ndt_.align(cur_pose_m_);
  //  fitness_score_ = cpu_ndt_.getFitnessScore();
  //  final_transformation_ = cpu_ndt_.getFinalTransformation();
  //  has_converged_ = cpu_ndt_.hasConverged();
  //  final_iters_ = cpu_ndt_.getFinalNumIteration();

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud_(new pcl::PointCloud<pcl::PointXYZI>);
  ndt_omp_->setInputSource(pc_source_);
  ndt_omp_->align(*aligned_cloud_, cur_pose_m_);
  fitness_score_ = ndt_omp_->getFitnessScore();
  final_transformation_ = ndt_omp_->getFinalTransformation();
  has_converged_ = ndt_omp_->hasConverged();
  final_iters_ = ndt_omp_->getFinalNumIteration();

  Eigen::Quaternionf tmp_q(final_transformation_.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  cur_pose_ndt_.pose.pose.position.x = final_transformation_(0, 3);
  cur_pose_ndt_.pose.pose.position.y = final_transformation_(1, 3);
  cur_pose_ndt_.pose.pose.position.z = final_transformation_(2, 3);
  cur_pose_ndt_.pose.pose.orientation.w = tmp_q.w();
  cur_pose_ndt_.pose.pose.orientation.x = tmp_q.x();
  cur_pose_ndt_.pose.pose.orientation.y = tmp_q.y();
  cur_pose_ndt_.pose.pose.orientation.z = tmp_q.z();
  cur_pose_ndt_.header.stamp = msg->header.stamp;
  cur_pose_m_ = final_transformation_;

  // 发布里程计位姿
  nav_msgs::Odometry updated_msg;
  updated_msg.header.stamp = msg->header.stamp;
  updated_msg.header.frame_id = "map";
  updated_msg.pose = cur_pose_ndt_.pose;
  pub_updated_pose_.publish(updated_msg);

  tf::Transform tf_m2b;
  tf_m2b.setOrigin(
      tf::Vector3(final_transformation_(0, 3), final_transformation_(1, 3), final_transformation_(2, 3)));
  tf_m2b.setRotation(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w()));

  if (use_odom_ && odom_ptr_last_ != -1) {
    tf_o2b_.setOrigin(tf::Vector3(odom_queue_[odom_ptr_front_].pose.pose.position.x,
                                  odom_queue_[odom_ptr_front_].pose.pose.position.y,
                                  odom_queue_[odom_ptr_front_].pose.pose.position.z));
    tf_o2b_.setRotation(tf::Quaternion(odom_queue_[odom_ptr_front_].pose.pose.orientation.x,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.y,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.z,
                                       odom_queue_[odom_ptr_front_].pose.pose.orientation.w));
    tf_m2o_ = tf_m2b * tf_o2b_.inverse();
    // tf::StampedTransform tmp;
    // tf_listener_.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.05));
    // tf_listener_.lookupTransform("/odom", "/base_link", ros::Time(0), tmp);
    // tf_m2o_ = tf_m2b * tmp.inverse();
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2o_, msg->header.stamp, "map", "/odom"));
  } else {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2b, msg->header.stamp, "map", "/base_link"));
  }

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "scan points size: " << pc_source_->size() << " points." << std::endl;
  std::cout << "map size: " << pc_target_->points.size() << " points." << std::endl;
  std::cout << "Fitness score: " << fitness_score_ << std::endl;
  std::cout << "iteration: " << final_iters_ << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << final_transformation_ << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(final_transformation_(0, 3) - pre_pose_m_(0, 3), 2) +
      std::pow(final_transformation_(1, 3) - pre_pose_m_(1, 3), 2) +
      std::pow(final_transformation_(2, 3) - pre_pose_m_(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  // 保存位姿
  if (saveKeyframesAndFactor()) {
    // 更新pc_source_, 对齐到地图上
    pcl::PointCloud<PointT>::Ptr pc_m(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*pc_source_, *pc_m, cur_pose_m_);
    //cpu_ndt_.updateVoxelGrid(pc_m);
    //ndt_omp_.r
  } else {
    std::cout << "too close" << std::endl;
  }

  // 实时的点云发布
  if (pub_current_frames_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr pc_msg(new sensor_msgs::PointCloud2);
    pcl::PointCloud<PointT>::Ptr pc_current(new pcl::PointCloud<PointT>());
    pc_current = transformPointCloud(tmp_cloud,
                                     cloud_keyposes_6d_->points[cloud_keyframes_.size() - 1]);

    pcl::toROSMsg(*pc_current, *pc_msg);
    pc_msg->header.frame_id = "map";
    pub_current_frames_.publish(*pc_msg);
  }

  // 更新位姿态
  pre_pose_m_ = cur_pose_m_;
  pre_pose_ndt_ = cur_pose_ndt_;

  // 匹配完了去看一下是否检测到回环，有的话更新当前pose
  correctPoses();*/
}

void LoopDetection::pcCB2(const sensor_msgs::PointCloud2ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  mutex_lock.lock();
  cloudBuf.push(*msg);
  mutex_lock.unlock();
}

void LoopDetection::imuCB(const sensor_msgs::ImuConstPtr &msg) {
  //  取姿态，是ENU数据吗？
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  // 加速度
  float acc_x = msg->linear_acceleration.x + 9.81 * sin(pitch);
  float acc_y = msg->linear_acceleration.y - 9.81 * cos(pitch) * sin(roll);
  float acc_z = msg->linear_acceleration.z - 9.81 * cos(pitch) * cos(roll);

  // imu队列里面装100条数据
  imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_queue_len_;

  if ((imu_ptr_last_ + 1) % imu_queue_len_ == imu_ptr_front_) {
    imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
  }

  // 维护几个队列的imu数据
  imu_time_[imu_ptr_last_] = msg->header.stamp.toSec();
  imu_roll_[imu_ptr_last_] = roll;
  imu_pitch_[imu_ptr_last_] = pitch;
  imu_yaw_[imu_ptr_last_] = yaw;
  imu_acc_x_[imu_ptr_last_] = acc_x;
  imu_acc_y_[imu_ptr_last_] = acc_y;
  imu_acc_z_[imu_ptr_last_] = acc_z;
  imu_angular_velo_x_[imu_ptr_last_] = msg->angular_velocity.x;
  imu_angular_velo_y_[imu_ptr_last_] = msg->angular_velocity.y;
  imu_angular_velo_z_[imu_ptr_last_] = msg->angular_velocity.z;

  // 转换到 imu 的全局坐标系中
  Eigen::Matrix3f rot = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y,
                                           msg->orientation.z).toRotationMatrix();
  Eigen::Vector3f acc = rot * Eigen::Vector3f(acc_x, acc_y, acc_z);
  // TODO: lego_loam 里没有对角速度转换，是否需要尚且存疑
  // Eigen::Vector3f angular_velo = rot * Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3f angular_velo(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  //  这里是记录imu在这个队列数据时间内的偏移？
  int imu_ptr_back = (imu_ptr_last_ - 1 + imu_queue_len_) % imu_queue_len_;
  double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
  if (time_diff < 1) {
    imu_shift_x_[imu_ptr_last_] = imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff +
        acc(0) * time_diff * time_diff * 0.5;
    imu_shift_y_[imu_ptr_last_] = imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff +
        acc(1) * time_diff * time_diff * 0.5;
    imu_shift_z_[imu_ptr_last_] = imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff +
        acc(2) * time_diff * time_diff * 0.5;

    imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
    imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
    imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

    imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) * time_diff;
    imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) * time_diff;
    imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) * time_diff;
  }
}

void LoopDetection::loopClosureThread() {
  // 不允许回环检测
  if (!loop_closure_enabled_) {
    return;
  }
  // 不停地进行回环检测
  ros::Duration duration(1);
  while (ros::ok()) {
    performLoopClosure();
    duration.sleep();
  }
}

/**
 * @brief 回环检测及位姿图更新
 * ICP 匹配添加回环约束
 */
void LoopDetection::performLoopClosure() {
  // 保证有关键帧
  if (cloud_keyposes_3d_->points.empty()) {
    return;
  }
  // 持续检测回环,未检测到就退出
  if (!detectLoopClosure()) {
    return;
  } else {
    ROS_WARN("detected loop closure");
  }

  // 检测到回环后，用icp去匹配
  auto start = std::chrono::system_clock::now();

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  // 注意input是当前帧
  icp.setInputSource(latest_keyframe_);   // 已经转换到map上的当前帧点云
  icp.setInputTarget(near_history_keyframes_); // 已经拼好的回环帧的localmap
  pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());

  //  初始位姿哪里来，就是当前的位姿
  Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
  initial_guess.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].roll, Eigen::Vector3f::UnitX()))
      .toRotationMatrix();
  initial_guess(0, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].x;
  initial_guess(1, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].y;
  initial_guess(2, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].z;
  icp.align(*unused_result);

  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();
  //Eigen::Matrix4f correction_frame = icp.getFinalTransformation();
  Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
  Eigen::Quaternionf tmp_q(icp_trans.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  if (has_converged == false || fitness_score > history_fitness_score_) {
    ROS_WARN("loop cannot closed");
    return;
  } else {
    ROS_WARN("loop closed");
    // 回环检测关键帧
    //    if (pub_icp_keyframes_.getNumSubscribers() > 0) {
    //      pcl::PointCloud<PointT>::Ptr closed_cloud(new pcl::PointCloud<PointT>());
    //      pcl::transformPointCloud(*latest_keyframe_, *closed_cloud, correction_frame);
    //      sensor_msgs::PointCloud2 msg;
    //      pcl::toROSMsg(*closed_cloud, msg);
    //      msg.header.stamp.fromSec(cloud_keyposes_6d_->points[latest_history_frame_id_].time);
    //      msg.header.frame_id = "map";
    //      pub_icp_keyframes_.publish(msg);
    //    }
  }

  Eigen::Matrix4f t_wrong = initial_guess;
  Eigen::Matrix4f T_correct = icp_trans * t_wrong;
  // Eigen::Matrix4f t_correct = correction_frame;
  Eigen::Quaternionf R_correct(T_correct.block<3, 3>(0, 0));

  // 更新因子图
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(R_correct.w(), R_correct.x(), R_correct.y(), R_correct.z()),
                                 Point3(T_correct(0, 3), T_correct(1, 3), T_correct(2, 3)));
  // closest_history_frame_id_是找到的回环帧id
  gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(cloud_keyposes_6d_->points[closest_history_frame_id_].roll,
                                            cloud_keyposes_6d_->points[closest_history_frame_id_].pitch,
                                            cloud_keyposes_6d_->points[closest_history_frame_id_].yaw),
                               Point3(cloud_keyposes_6d_->points[closest_history_frame_id_].x,
                                      cloud_keyposes_6d_->points[closest_history_frame_id_].y,
                                      cloud_keyposes_6d_->points[closest_history_frame_id_].z));
  float noise_score = fitness_score;
  gtsam::Vector vector6(6);
  vector6 << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;
  constraint_noise_ = noiseModel::Diagonal::Variances(vector6);

  // 添加约束边
  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph_.add(BetweenFactor<Pose3>(latest_history_frame_id_, closest_history_frame_id_, pose_from.between(pose_to),
                                       constraint_noise_));
  isam->update(gtSAMgraph_);
  isam->update();
  gtSAMgraph_.resize(0);

  std::cout << "---------------------------LOOP CLOSE!!!-------------------------------------------" << std::endl;
  //  std::cout << "Time elapsed: " << elapsed.count() << std::endl;
  std::cout << "Number of source points: " << latest_keyframe_->size() << " points." << std::endl;
  std::cout << "target cloud size: " << near_history_keyframes_->points.size() << " points." << std::endl;
  std::cout << "ICP has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << T_correct << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(icp_trans(0, 3) - initial_guess(0, 3), 2) +
      std::pow(icp_trans(1, 3) - initial_guess(1, 3), 2) +
      std::pow(icp_trans(2, 3) - initial_guess(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  loop_closed_ = true;
}

/**
 * @brief 基于里程计的回环检测，直接提取当前位姿附近(radiusSearch)的 keyframe 作为 icp 的 target
 *
 */
bool LoopDetection::detectLoopClosure() {
  // near_history_keyframes_是附近的关键帧
  latest_keyframe_->clear();
  near_history_keyframes_->clear();

  std::lock_guard<std::mutex> lock(mtx_);

  PointT cur_pose;
  cur_pose.x = cur_pose_m_(0, 3);
  cur_pose.y = cur_pose_m_(1, 3);
  cur_pose.z = cur_pose_m_(2, 3);
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  kdtree_poses_->radiusSearch(cur_pose, history_search_radius_, search_idx_, search_dist_);

  // kdtree找到距离当前位置最近的历史关键帧
  latest_history_frame_id_ = cloud_keyframes_.size() - 1;
  closest_history_frame_id_ = -1;
  for (int i = 0; i < search_idx_.size(); ++i) {
    // 历史帧与当前帧必须间隔时间足够远
    if (cur_pose_ndt_.header.stamp.toSec() - cloud_keyposes_6d_->points[search_idx_[i]].time > 20) {
      closest_history_frame_id_ = search_idx_[i];
      break;
    }
  }
  if (closest_history_frame_id_ == -1) {
    return false;
  }

  // 复制一份找到的回环帧
  pcl::PointCloud<PointT>::Ptr tmp_ptr(new pcl::PointCloud<PointT>());
  tmp_ptr = transformPointCloud(cloud_keyframes_[latest_history_frame_id_],
                                cloud_keyposes_6d_->points[latest_history_frame_id_]);
  pcl::copyPointCloud(*tmp_ptr, *latest_keyframe_);

  // 把回环帧附近前后20帧拼接成localmap
  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i) {
    j = closest_history_frame_id_ + i;
    if (j < 0 || j >= latest_history_frame_id_) {
      continue;
    }
    *tmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
  }

  ds_history_keyframes_.setInputCloud(tmp_cloud);
  ds_history_keyframes_.filter(*near_history_keyframes_);

  // 发布回环帧的localmap
/*  if (pub_history_keyframes_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*near_history_keyframes_, msg);
    msg.header.stamp = cur_pose_ndt_.header.stamp;
    msg.header.frame_id = "map";
    pub_history_keyframes_.publish(msg);
  }*/

  return true;
}

/**
 * @brief 检测出回环后，更新位姿及 target map
 *
 */
void LoopDetection::correctPoses() {
  // 检测到回环了在下一帧中更新全局位姿 cloud_keyposes_6d_和cloud_keyposes_3d_
  if (loop_closed_) {
    recent_keyframes_.clear();
    ROS_WARN("corret loop pose");
    int num_poses = isam_current_estimate_.size();
    for (int i = 0; i < num_poses; ++i) {
      cloud_keyposes_6d_->points[i].x = cloud_keyposes_3d_->points[i].x =
          isam_current_estimate_.at<Pose3>(i).translation().x();
      cloud_keyposes_6d_->points[i].y = cloud_keyposes_3d_->points[i].y =
          isam_current_estimate_.at<Pose3>(i).translation().y();
      cloud_keyposes_6d_->points[i].z = cloud_keyposes_3d_->points[i].z =
          isam_current_estimate_.at<Pose3>(i).translation().z();
      cloud_keyposes_6d_->points[i].roll = isam_current_estimate_.at<Pose3>(i).rotation().roll();
      cloud_keyposes_6d_->points[i].pitch = isam_current_estimate_.at<Pose3>(i).rotation().pitch();
      cloud_keyposes_6d_->points[i].yaw = isam_current_estimate_.at<Pose3>(i).rotation().yaw();
    }
    loop_closed_ = false;
  }
}
