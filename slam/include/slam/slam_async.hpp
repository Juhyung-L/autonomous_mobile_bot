/*
 * slam
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

/* Author: Steven Macenski */

#ifndef SLAM__SLAM_ASYNC_HPP_
#define SLAM__SLAM_ASYNC_HPP_

#include <memory>
#include "slam/slam_common.hpp"

namespace slam
{

class AsynchronousSlam : public Slam
{
public:
  explicit AsynchronousSlam(rclcpp::NodeOptions options);
  ~AsynchronousSlam() {}

protected:
  void laserCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam::srv::DeserializePoseGraph::Response> resp) override;
};

}  // namespace slam

#endif  // SLAM__SLAM_ASYNC_HPP_
