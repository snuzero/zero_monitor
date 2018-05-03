#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "core_msgs/PathArray.h"
#include "core_msgs/VehicleState.h"
#include "core_msgs/CenPoint.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>

#define Z_DEBUG true

struct vehicle_state {
  bool is_auto;
  bool estop;
  int flag_obstacle;
  int gear;
  int brake;
  float speed;
  float steer;
};

std::string config_path;

cv::VideoWriter outputMonitorVideo;

vehicle_state vState;

image_transport::Publisher publishMonitorImg;
sensor_msgs::ImagePtr msgMonitorImg;
int map_width, map_height;
float map_res;
int x_waypoint, y_waypoint;
cv::Mat occupancy_map;
double occupancy_map_timestamp;
cv::Mat lane_topview;
double lane_topview_timestamp;

cv::Mat state_monitor;
cv::Mat monitor_img;
std::vector<geometry_msgs::Vector3> path_points;
double pathtracking_timestamp;

boost::mutex map_mutex_;


void callbackLaneCam(const sensor_msgs::ImageConstPtr& msg_lane_cam)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_lane_cam, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  cv::Mat temp = cv_ptr->image.clone();
  cv::Mat temp_resized;
  cv::resize(temp, temp_resized, cv::Size(map_height*2,map_width*2),0,0);

  cv::transpose(temp_resized, temp_resized);
  cv::Mat rot_matrix = getRotationMatrix2D(cv::Point(map_height, map_width), 180, 1.0);
  cv::warpAffine(temp_resized, lane_topview, rot_matrix, temp_resized.size());

  // lane_topview = cv_ptr->image.clone();
  lane_topview_timestamp = msg_lane_cam->header.stamp.toSec();
}
void callbackOccupancyRaw(const sensor_msgs::ImageConstPtr& msg_occupancy_raw)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_occupancy_raw, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  state_monitor = cv_ptr->image.clone();
}

void callbackOccupancyMap(const sensor_msgs::ImageConstPtr& msg_occupancy_map)
{
  //if(Z_DEBUG) std::cout<<"occupanc map callback!"<<std::endl;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_occupancy_map, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  occupancy_map = cv_ptr->image.clone();
  occupancy_map_timestamp = msg_occupancy_map->header.stamp.toSec();
}

void callbackWaypoint(const core_msgs::CenPointConstPtr& msg_waypoint) {
  x_waypoint = (float)(msg_waypoint->x_waypoint)/map_res;
  y_waypoint = (float)(msg_waypoint->y_waypoint)/map_res;
}

void callbackFlagObstacle(const std_msgs::Int32::ConstPtr & msg_flag_obstacle) {
  vState.flag_obstacle = msg_flag_obstacle->data;
}

void callbackPath(const core_msgs::PathArrayConstPtr& msg_path_tracking)
{
  if(Z_DEBUG) std::cout<<"path callback!"<<std::endl;

  path_points = msg_path_tracking->pathpoints;
  pathtracking_timestamp = msg_path_tracking->header.stamp.toSec();
}

void callbackState(const core_msgs::VehicleStateConstPtr& msg_state) {
  vState.is_auto = msg_state->is_auto;
  vState.estop = msg_state->estop;
  vState.gear = msg_state->gear;
  vState.brake = msg_state->brake;
  vState.steer = msg_state->steer;
  vState.speed = msg_state->speed;
}

void callbackTerminate(const std_msgs::Int32Ptr& record){
  outputMonitorVideo.release();
  ROS_INFO("Monitor Video recording safely terminated");
  ros::shutdown();
  return;
}

int main(int argc, char** argv)
{
  std::string record_path = ros::package::getPath("zero_monitor");
  //TODO: add date&time to the file name
  record_path += "/data/monitor.mp4";
  ROS_INFO_STREAM(record_path);

  x_waypoint = 180;
  y_waypoint = 0;

  //TODO: change root path later
  config_path = ros::package::getPath("map_generator");
  config_path += "/config/system_config.yaml";
  cv::FileStorage params_config(config_path, cv::FileStorage::READ);
  map_width = params_config["Map.width"];
  map_height = params_config["Map.height"];
  map_res = params_config["Map.resolution"];
  bool isVideoOpened =  outputMonitorVideo.open(record_path, CV_FOURCC('X', '2', '6', '4'), 25, cv::Size(map_width*6, map_height*2+100), true);

  if(isVideoOpened)
    ROS_INFO("monitor video starts recorded!");

  occupancy_map = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  lane_topview = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  state_monitor = cv::Mat::zeros(map_height,map_width,CV_8UC3);

  ros::init(argc, argv, "map_generator");
  ros::start();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  publishMonitorImg = it.advertise("/monitor",1);
  msgMonitorImg.reset(new sensor_msgs::Image);

  ros::Subscriber laneSub = nh.subscribe("/warped_image",1,callbackLaneCam);
  ros::Subscriber occupancySub = nh.subscribe("/occupancy_map",1,callbackOccupancyMap);
  ros::Subscriber occupancyRawSub = nh.subscribe("/occupancy_map_raw",1,callbackOccupancyRaw);
  ros::Subscriber pathSub;
  if(Z_DEBUG) pathSub = nh.subscribe("/sPath",1,callbackPath);
  else pathSub = nh.subscribe("/path_tracking",1,callbackPath);
  ros::Subscriber waypointSub = nh.subscribe("/waypoints",1,callbackWaypoint);
  ros::Subscriber flagobstacleSub = nh.subscribe("/flag_obstacle",1,callbackFlagObstacle);
  ros::Subscriber stateSub = nh.subscribe("/vehicle_state",1,callbackState);

  ros::Subscriber endSub = nh.subscribe("/end_system",1,callbackTerminate);
  ros::Rate loop_rate(10);
  while(ros::ok()){
    //if(Z_DEBUG) std::cout<<"while loop starts!!"<<std::endl;

    //monitor initialization
    monitor_img = cv::Mat::zeros(map_height*2+100, map_width*6,CV_8UC3);
    //if(Z_DEBUG) std::cout<<"monitor img created!!"<<std::endl;
    //cv::imshow("occupancy map",occupancy_map);

    cv::Mat occupancy_map_resized = cv::Mat::zeros(map_height*2,map_width*2,CV_8UC3);
    cv::resize(occupancy_map, occupancy_map_resized, cv::Size(map_height*2,map_width*2),0,0);
    //if(Z_DEBUG) std::cout<<"occupancy map resized!!"<<std::endl;

    cv::Mat lane_topview_resized = cv::Mat::zeros(map_height*2,map_width*2,CV_8UC3);
    cv::resize(lane_topview, lane_topview_resized, cv::Size(map_height*2,map_width*2),0,0);
    //if(Z_DEBUG) std::cout<<"lane topview resized!!"<<std::endl;

    cv::circle(state_monitor, cv::Point(map_width/2 - y_waypoint, map_height-x_waypoint),3,cv::Scalar(255,255,100), -1);
    cv::Mat state_monitor_resized = cv::Mat::zeros(map_height*2,map_width*2,CV_8UC3);
    cv::resize(state_monitor, state_monitor_resized, cv::Size(map_height*2,map_width*2),0,0);
    //if(Z_DEBUG) std::cout<<"state monitor resized!!"<<std::endl;

    //TODO: add path to state_monitor_resized
    if(vState.flag_obstacle > 0) {
      for(int i= path_points.size()-1; i>=1;i--){
        int start_x = (int)(2*path_points.at(i).x);
        int start_y = (int)(2*path_points.at(i).y);
        int end_x = (int)(2*path_points.at(i-1).x);
        int end_y = (int)(2*path_points.at(i-1).y);
        cv::line(state_monitor_resized,cv::Point(start_x, start_y), cv::Point(end_x, end_y), cv::Scalar(100,255,100),2);
      }
    }
    else if(vState.flag_obstacle == 0) {
      cv::line(state_monitor_resized,cv::Point(map_width, map_height*2), cv::Point(map_width - 2*y_waypoint, map_height*2-2*x_waypoint), cv::Scalar(255,255,100),2);
    }

    //TODO: save path image to other directory (in image files)
    ///add above three images to the monitor_img
    lane_topview_resized.copyTo(monitor_img(cv::Rect(0,0,lane_topview_resized.cols,lane_topview_resized.rows)));
    //if(Z_DEBUG) std::cout<<"lane topview copied!!"<<std::endl;
    state_monitor_resized.copyTo(monitor_img(cv::Rect(lane_topview_resized.cols,0,state_monitor_resized.cols,state_monitor_resized.rows)));
    //if(Z_DEBUG) std::cout<<"state monitor copied!!"<<std::endl;
    occupancy_map_resized.copyTo(monitor_img(cv::Rect(lane_topview_resized.cols+state_monitor_resized.cols,0,occupancy_map_resized.cols,occupancy_map_resized.rows)));
    //if(Z_DEBUG) std::cout<<"occupancy map copied!!"<<std::endl;
    //TODO: add text info to the image

    //publishing monitor image
    msgMonitorImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", monitor_img).toImageMsg();
    publishMonitorImg.publish(msgMonitorImg);

    //adding it to the video record
    outputMonitorVideo << monitor_img;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
