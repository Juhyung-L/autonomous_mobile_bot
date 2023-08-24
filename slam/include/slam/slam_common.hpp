/*
 * slam
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#ifndef SLAM__SLAM_COMMON_HPP_
#define SLAM__SLAM_COMMON_HPP_

#include <sys/resource.h>
#include <boost/thread.hpp>
#include <string>
#include <map>
#include <vector>
#include <queue>
#include <cstdlib>
#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "pluginlib/class_loader.hpp"

#include "slam/toolbox_types.hpp"
#include "slam/slam_mapper.hpp"
#include "slam/snap_utils.hpp"
#include "slam/laser_utils.hpp"
#include "slam/get_pose_helper.hpp"
#include "slam/map_saver.hpp"
#include "slam/loop_closure_assistant.hpp"
#include "karto_sdk/Karto.h"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"


namespace slam
{

// dirty, dirty cheat I love
using namespace ::toolbox_types;  // NOLINT
using namespace ::karto;  // NOLINT

class Slam : public nav2_util::LifecycleNode
{
public:
  explicit Slam(rclcpp::NodeOptions);
  Slam();
  virtual ~Slam();
  // virtual void configure();
  virtual void loadPoseGraphByParams();

protected:
  /*
   * @brief Lifecycle configure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle activate
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle deactivate
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle cleanup
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle shutdown
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // threads
  void publishVisualizations();
  void publishTransformLoop(const double & transform_publish_period);

  // setup
  void setParams();
  void setSolver();
  void setROSInterfaces();

  // callbacks
  virtual void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) = 0;
  bool mapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> res);
  virtual bool serializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam::srv::SerializePoseGraph::Request> req,
    std::shared_ptr<slam::srv::SerializePoseGraph::Response> resp);
  virtual bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam::srv::DeserializePoseGraph::Response> resp);

  // Loaders
  void loadSerializedPoseGraph(std::unique_ptr<karto::Mapper> &, std::unique_ptr<karto::Dataset> &);

  // functional bits
  karto::LaserRangeFinder * getLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
  virtual karto::LocalizedRangeScan * addScan(
    karto::LaserRangeFinder * laser, const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    karto::Pose2 & karto_pose);
  karto::LocalizedRangeScan * addScan(karto::LaserRangeFinder * laser, PosedScan & scanWPose);
  bool updateMap();
  tf2::Stamped<tf2::Transform> setTransformFromPoses(
    const karto::Pose2 & pose,
    const karto::Pose2 & karto_pose, const rclcpp::Time & t,
    const bool & update_reprocessing_transform);
  karto::LocalizedRangeScan * getLocalizedRangeScan(
    karto::LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    karto::Pose2 & karto_pose);
  bool shouldStartWithPoseGraph(
    std::string & filename, geometry_msgs::msg::Pose2D & pose,
    bool & start_at_dock);
  bool shouldProcessScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    const karto::Pose2 & pose);
  void publishPose(
    const Pose2 & pose,
    const Matrix3 & cov,
    const rclcpp::Time & t);

  // pausing bits
  bool isPaused(const PausedApplication & app);
  bool pauseNewMeasurementsCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam::srv::Pause::Request> req,
    std::shared_ptr<slam::srv::Pause::Response> resp);

  // ROS-y-ness
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfL_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>> scan_filter_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;
  std::shared_ptr<rclcpp::Service<nav_msgs::srv::GetMap>> ssMap_;
  std::shared_ptr<rclcpp::Service<slam::srv::Pause>> ssPauseMeasurements_;
  std::shared_ptr<rclcpp::Service<slam::srv::SerializePoseGraph>> ssSerialize_;
  std::shared_ptr<rclcpp::Service<slam::srv::DeserializePoseGraph>> ssDesserialize_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>> sst_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::MapMetaData>> sstm_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose_pub_;

  // Storage for ROS parameters
  std::string odom_frame_, map_frame_, base_frame_, map_name_, scan_topic_;
  rclcpp::Duration transform_timeout_, minimum_time_interval_;
  std_msgs::msg::Header scan_header;
  int throttle_scans_, scan_queue_size_;

  double resolution_;
  double position_covariance_scale_;
  double yaw_covariance_scale_;
  bool first_measurement_, enable_interactive_mode_;
  bool continue_threads_;
  double transform_publish_period;
  double tf_buffer_duration;
  double map_update_interval;
  std::string solver_plugin;

  rclcpp::ParameterValue map_start_pose;
  rclcpp::ParameterValue map_start_at_dock;

  // Book keeping
  std::unique_ptr<mapper_utils::SMapper> smapper_;
  std::unique_ptr<karto::Dataset> dataset_;
  std::map<std::string, laser_utils::LaserMetadata> lasers_;

  // helpers
  std::unique_ptr<laser_utils::LaserAssistant> laser_assistant_;
  std::unique_ptr<pose_utils::GetPoseHelper> pose_helper_;
  std::unique_ptr<map_saver::MapSaver> map_saver_;
  std::unique_ptr<loop_closure_assistant::LoopClosureAssistant> closure_assistant_;
  std::unique_ptr<laser_utils::ScanHolder> scan_holder_;

  // Internal state
  std::vector<std::unique_ptr<boost::thread>> threads_;
  tf2::Transform map_to_odom_;
  boost::mutex map_to_odom_mutex_, smapper_mutex_, pose_mutex_;
  PausedState state_;
  nav_msgs::srv::GetMap::Response map_;
  ProcessType processor_type_;
  std::unique_ptr<karto::Pose2> process_near_pose_;
  tf2::Transform reprocessing_transform_;

  // pluginlib
  pluginlib::ClassLoader<karto::ScanSolver> solver_loader_;
  std::shared_ptr<karto::ScanSolver> solver_;
};

}  // namespace slam

#endif   // slam__slam_COMMON_HPP_
