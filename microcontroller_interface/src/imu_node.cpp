#include <memory>
#include <string>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace LibSerial;

constexpr const char* const SERIAL_PORT_1 = "/dev/ttyUSB0";

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_node");
    auto imu_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/imu", 10);
    auto tf_B = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    SerialPort serial_port;

    try
    {
        serial_port.Open(SERIAL_PORT_1);
    } 
    catch (const OpenFailed&) 
    {
        std::cerr << "The serial port did not open correctly." << std::endl;
        return EXIT_FAILURE;
    }

    serial_port.SetBaudRate(BaudRate::BAUD_38400);
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serial_port.SetParity(Parity::PARITY_NONE);
    serial_port.SetStopBits(StopBits::STOP_BITS_1);

    std::string line;
    size_t timeout_ms = 3000; // 3 seconds before timeout

    geometry_msgs::msg::PoseStamped p;
    geometry_msgs::msg::TransformStamped t;
    p.header.frame_id = "/map";
    t.header.frame_id = "/map";
    t.child_frame_id = "/imu";
    while (rclcpp::ok()) 
    {
        try 
        {
            serial_port.ReadLine(line, '\n', timeout_ms); // read until the '\n' character
            std::stringstream ss(line);
            
            // print the line if one of the tokens is not a number
            if (!(ss >> p.pose.orientation.x &&
                  ss >> p.pose.orientation.y &&
                  ss >> p.pose.orientation.z &&
                  ss >> p.pose.orientation.w))
            {
                std::cout << line;
                continue;
            }
            
            p.header.stamp = node->now();
            imu_pub->publish(p);
            
            t.transform.rotation.x = p.pose.orientation.x;
            t.transform.rotation.y = p.pose.orientation.y;
            t.transform.rotation.z = p.pose.orientation.z;            
            t.transform.rotation.w = p.pose.orientation.w;

            t.header.stamp = node->now();
            tf_B->sendTransform(t);
        } 
        catch (const ReadTimeout&) 
        {
            std::cerr << "The ReadByte() call has timed out." << std::endl;
        }
    }

    // Successful program completion.
    std::cout << "The example program successfully completed!" << std::endl;
    rclcpp::shutdown();
    return 0;
}