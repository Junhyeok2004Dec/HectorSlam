 //=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================https://github.com/Junhyeok2004Dec/HectorSlam.git

#include <fstream>
#include <vector>

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#ifndef TF_SCALAR_H
  typedef btScalar tfScalar;
#endif


#define ORIGIN_RANGE 2.0 // 원점으로 인식되는 거리 : 2.0m 이내 진입 시



int lap = 0; // 바퀴수

std::string mapList[] = {"map1s", "map2s", "map3s", "map4s", "map5s", "map6s",
   "map7s", "map8s", "map9s", "map10s", "map11s"}; 
std::string mapTopic_;

std::vector<std::string> csvData; // waypoint 데이타
float distance_from_Origin; // 초기 지점과의 거리 -> used loop closure
bool isCenter = false; 
bool changelap = false;
//Save2CSV
// string
void saveToCsv(const std::string& filePath, const std::string data) {
    std::fstream outFile(filePath, std::ios::app);

    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filePath << std::endl;
        return;
    }

    outFile << data;
    outFile << std::endl; // EOL

    outFile.close();
  
}
// array
void saveToCsv(const std::string& filePath, const std::vector<std::string>& data) {
    sleep(0.1);
    std::fstream outFile(filePath, std::ios::app);
    
    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filePath << std::endl;
        return;
    }


    for (size_t i = 0; i < data.size(); ++i) {
        outFile << data[i];
        if (i != data.size() - 1) { 
            outFile << ",";
        }
    }
    outFile << std::endl; // EOL

    
    outFile.close();
}

HectorMappingRos::HectorMappingRos()
  : debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , map__publish_thread_(0)
  , initial_pose_set_(true)
  , pause_scan_processing_(false)
{
  ros::NodeHandle private_nh_("~");

  lapPublisher = private_nh_.advertise<std_msgs::String>("/lap", 10);



  //std::string mapTopic_ = "map1s";// -> 다중 map을 이용하도록 할 것.

  //for(int i = 0; i < 11; i++) mapTopic_ = mapList[i];
  //
  mapTopic_ = "map1s";
  

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);
  private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
  private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);  
  private_nh_.param("map_size", p_map_size_, 1024); 
  private_nh_.param("map_start_x", p_map_start_x_, 0.0);
  private_nh_.param("map_start_y", p_map_start_y_, 0.0); 
  private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
  private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

  private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
  private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
  private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));
  private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

  private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
  private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

  private_nh_.param("output_timing", p_timing_output_,false);

  private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  private_nh_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  private_nh_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output_)
  {
    ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
  }


// 해당 slamProcesor로부터 Slamposition이 갱신되고 있는지 확인해야 한다.


  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  //mapLevels = 2;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if (i==0){
    mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
    }

    

  }

  // Initialize services
  reset_map_service_ = node_.advertiseService("reset_map", &HectorMappingRos::resetMapCallback, this);
  restart_hector_service_ = node_.advertiseService("restart_mapping_with_new_pose", &HectorMappingRos::restartHectorCallback, this);
  toggle_scan_processing_service_ = node_.advertiseService("pause_mapping", &HectorMappingRos::pauseMapCallback, this);

  ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this);
  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 2, &HectorMappingRos::sysMsgCallback, this);


  // Hector slam에서 posePublish을 담당하는 부분
  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
  posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false); 
  
  scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(node_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));


  map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);
  
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
    delete map__publish_thread_;
}


void HectorMappingRos::sysMsgCallback(const std_msgs::String& string)
{
  ROS_INFO("HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    ROS_INFO("HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                   nav_msgs::GetMap::Response &res)
{
  ROS_INFO("HectorSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

bool HectorMappingRos::resetMapCallback(std_srvs::Trigger::Request  &req,
                                        std_srvs::Trigger::Response &res)
{
  ROS_INFO("HectorSM Reset map service called");
  slamProcessor->reset();
  return true;
}

bool HectorMappingRos::restartHectorCallback(hector_mapping::ResetMapping::Request  &req,
                                             hector_mapping::ResetMapping::Response &res)
{
  // Reset map
  ROS_INFO("HectorSM Reset map");
  slamProcessor->reset();

  // Reset pose
  this->resetPose(req.initial_pose);

  // Unpause node (in case it is paused)
  this->toggleMappingPause(false);

  // Return success
  return true;
}

bool HectorMappingRos::pauseMapCallback(std_srvs::SetBool::Request  &req,
                                        std_srvs::SetBool::Response &res)
{
  this->toggleMappingPause(req.data);
  res.success = true;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map_.map);
}

void HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{


  ROS_INFO("[DEBUG] SetServiceGetMapData");
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/


void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r(1.0 / map_pub_period);
  while(ros::ok())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime (ros::Time::now());
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    //ros::WallDuration t2 = ros::WallTime::now() - t1;

    //std::cout << "time s: " << t2.toSec();
    //ROS_INFO("HectorSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{

  ROS_INFO("[Debug] InitialPoseCallback");
  this->resetPose(msg->pose.pose);
}

void HectorMappingRos::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void HectorMappingRos::resetPose(const geometry_msgs::Pose &pose)
{
  initial_pose_set_ = true;
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, util::getYawFromQuat(pose.orientation));
  ROS_INFO("[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
           initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}

void HectorMappingRos::changeLapAndResetMap() {
    // lap을 증가시키고 새로운 lap에 대한 작업 수행
    lap++;


    // 현재까지의 맵을 저장
    std::string map_file_name = "/home/ak47/maps/map" + std::to_string(lap - 1) + "";
    saveCurrentMap(map_file_name);

    // Hector SLAM의 맵을 초기화하고, 새로운 lap을 시작
    resetSlamProcessor();

    // 이전 lap에서 저장한 맵을 불러와서 SLAM 프로세서에 로드
    std::string load_map_file = "/home/ak47/maps/map" + std::to_string(lap - 1) + "";
    loadPreviousMap(load_map_file);
}

void HectorMappingRos::saveCurrentMap(const std::string& map_file_name) {
    // 현재 맵을 저장하기 위한 로직을 구현합니다.
    // map_server를 사용하여 맵을 파일로 저장할 수 있습니다.
    ROS_INFO("Saving current map to %s", map_file_name.c_str());

    // 예시 코드: map_server를 호출하여 맵 저장
    system(("rosrun map_server map_saver -f " + map_file_name).c_str());
}

void HectorMappingRos::resetSlamProcessor() {
    // Hector SLAM 프로세서를 리셋하여 새로운 SLAM을 시작할 수 있도록 합니다.
    ROS_INFO("Resetting SLAM processor for new lap.");
    // 이 함수는 slamProcessor 객체를 리셋하는 로직을 포함해야 합니다.
    slamProcessor->reset();
}

void HectorMappingRos::loadPreviousMap(const std::string& map_file_name) {
    // 이전 lap에서 저장한 맵을 로드하여 SLAM 프로세서에 적용합니다.
    ROS_INFO("Loading previous map from %s", map_file_name.c_str());

    // 예시 코드: map_server를 사용하여 맵을 로드
    // 이 부분은 map_server와 상호작용하여 맵을 불러오는 로직을 구현합니다.
    system(("rosrun map_server map_server " + map_file_name).c_str());

    // 맵을 SLAM 프로세서에 적용하는 로직을 구현해야 합니다.
    // ...
}


void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
{
    if (pause_scan_processing_)
    {
        ROS_INFO("pause_scan_processing");
        return;
    }

    if (hectorDrawings)
    {
        hectorDrawings->setTime(scan.header.stamp);
    }

    ros::WallTime start_time = ros::WallTime::now();

    // 스캔 데이터를 처리하고 SLAM 프로세서를 업데이트합니다.
    if (!p_use_tf_scan_transformation_)
    {
        // tf 변환을 사용하지 않을 경우
        this->rosLaserScanToDataContainer(scan, laserScanContainer, slamProcessor->getScaleToMap());
        slamProcessor->update(laserScanContainer, slamProcessor->getLastScanMatchPose());
    }
    else
    {
        // tf 변환을 사용할 경우
        tf::StampedTransform laser_transform;
        try
        {
            tf_.waitForTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp, ros::Duration(0.5));
            tf_.lookupTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp, laser_transform);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Transform error: %s", ex.what());
            return;
        }

        // 레이저 스캔을 포인트 클라우드로 변환
        projector_.projectLaser(scan, laser_point_cloud_, 30.0);

        // 포인트 클라우드를 데이터 컨테이너로 변환
        this->rosPointCloudToDataContainer(laser_point_cloud_, laser_transform, laserScanContainer, slamProcessor->getScaleToMap());

        // SLAM 프로세서의 초기 위치 추정 설정
        // initilization
        Eigen::Vector3f start_estimate(Eigen::Vector3f::Zero());
        if (initial_pose_set_)
        {
            initial_pose_set_ = false;
            start_estimate = initial_pose_;
        }
        else if (p_use_tf_pose_start_estimate_)
        {
            try
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_map_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_map_frame_, p_base_frame_, scan.header.stamp, stamped_pose);

                const double yaw = tf::getYaw(stamped_pose.getRotation());
                start_estimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(), stamped_pose.getOrigin().getY(), yaw);
            }
            catch (tf::TransformException& e)
            {
                ROS_ERROR("Transform from %s to %s failed: %s", p_map_frame_.c_str(), p_base_frame_.c_str(), e.what());
                start_estimate = slamProcessor->getLastScanMatchPose();
            }
        }
        else
        {
            start_estimate = slamProcessor->getLastScanMatchPose();
        }

        // SLAM 프로세서 업데이트
        slamProcessor->update(laserScanContainer, start_estimate);
    }

    // **SLAM 위치 정보 업데이트**
    poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(),
                              slamProcessor->getLastScanMatchCovariance(),
                              scan.header.stamp,
                              p_map_frame_);

    
    // publish robot's position from slam
    poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
    posePublisher_.publish(poseInfoContainer_.getPoseStamped());

    
    csvData.push_back(std::to_string(poseInfoContainer_.getPoseStamped().pose.position.x));
    csvData.push_back(std::to_string(poseInfoContainer_.getPoseStamped().pose.position.y));
    csvData.push_back(std::to_string(tf::getYaw(poseInfoContainer_.getPoseStamped().pose.orientation)));

    // Update 된 output
    /*
    ROS_INFO("Updated pose: x=%f, y=%f, yaw=%f",
             poseInfoContainer_.getPoseStamped().pose.position.x,
             poseInfoContainer_.getPoseStamped().pose.position.y,
             tf::getYaw(poseInfoContainer_.getPoseStamped().pose.orientation));
  */
    saveToCsv("/home/ak47/waypoints/test.csv", csvData);

    csvData.erase(csvData.begin() + 0);
    csvData.erase(csvData.begin() + 1);
    csvData.erase(csvData.begin() + 2); // data 초기화



  //loop closure
  distance_from_Origin = sqrt(
    pow(poseInfoContainer_.getPoseStamped().pose.position.x,2) + pow(poseInfoContainer_.getPoseStamped().pose.position.y,2)
     );
    
    
  if (distance_from_Origin > ORIGIN_RANGE)  isCenter = false; 
  else isCenter = true;

  if(isCenter) {
    changelap = true;   
  }
  
  
  if(!isCenter && (changelap && ( distance_from_Origin > ORIGIN_RANGE + 1))) {
    // 중심에서 벗어난 뒤 바로 실행할 경우, 경계면에서 문제 발생 -ㅣ> 경계면에서 벗어난 이후에 add lap
    
     
      changelap = false;
      changeLapAndResetMap();
    
  }

 
  //ROS_INFO("isCenter : %d",isCenter);
  //ROS_INFO("changelap : %d",changelap);

  //ROS_INFO("lap : %d", lap);

  std_msgs::String lapData;
  lapData.data = std::to_string(lap);
  lapPublisher.publish(lapData);
}