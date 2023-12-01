#include <memory>
#include <string>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <cmath>

#include "libserial/SerialPort.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace LibSerial;

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("odom_imu_node");
    
    node->declare_parameter<std::string>("serial_port", "ttyUSB0");
    node->declare_parameter<double>("wheel_radius", 0.022);
    node->declare_parameter<double>("wheel_separation", 0.097);
    node->declare_parameter<std::string>("odom_header_frame", "odom");
    node->declare_parameter<std::string>("odom_child_frame", "base_footprint");
    node->declare_parameter<std::string>("imu_header_frame", "base_footprint");
    
    std::string serial_port_path = node->get_parameter("serial_port").as_string();
    double wheel_radius = node->get_parameter("wheel_radius").as_double();
    double wheel_separation = node->get_parameter("wheel_separation").as_double();
    std::string odom_header_frame = node->get_parameter("odom_header_frame").as_string();
    std::string odom_child_frame = node->get_parameter("odom_child_frame").as_string();
    std::string imu_header_frame = node->get_parameter("imu_header_frame").as_string();

    double wheel_circumference = 2.0 * M_PI * wheel_radius;

    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    auto ang_vel_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_vels", 10);

    SerialPort serial_port;

    try
    {
        serial_port.Open(serial_port_path.c_str());
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

    nav_msgs::msg::Odometry odom;
    sensor_msgs::msg::Imu imu;
    std_msgs::msg::Float64MultiArray ang_vels;

    odom.header.frame_id = odom_header_frame;
    odom.child_frame_id = odom_child_frame;
    imu.header.frame_id = imu_header_frame;

    double num_rev_r;
    double num_rev_l;
    double ang_vel_r;
    double ang_vel_l;
    double d;
    double dl;
    double dr;
    double d_yaw;
    double yaw = 0;

    std::vector<std::string> tokens;
    const int imu_encoder_token_size = 11;
    const int only_encoder_token_size = 4; 

    while (rclcpp::ok()) 
    {
        try 
        {
            serial_port.ReadLine(line, '\n', timeout_ms); // read until the '\n' character
            std::stringstream ss(line);
            std::string token;

            while (std::getline(ss, token, ','))
            {
                tokens.push_back(token);
            }

	    if (tokens.size() != imu_encoder_token_size && tokens.size() != only_encoder_token_size)
            {
                std::cout << line;
                tokens.clear();
		continue;
            }

            if (tokens.size() == imu_encoder_token_size) // both imu and encoder data
            {  
                num_rev_r = std::stod(tokens[0]);
                num_rev_l = std::stod(tokens[1]);
                ang_vel_r = std::stod(tokens[2]);
                ang_vel_l = std::stod(tokens[3]);
                imu.orientation.x = std::stod(tokens[4]);
                imu.orientation.y = std::stod(tokens[5]);
                imu.orientation.z = std::stod(tokens[6]);
                imu.orientation.w = std::stod(tokens[7]);
                imu.linear_acceleration.x = std::stod(tokens[8]);
                imu.linear_acceleration.y = std::stod(tokens[9]);
                imu.linear_acceleration.z = std::stod(tokens[10]);
	    }
            else if (tokens.size() == only_encoder_token_size) // only encoder data
            {
                num_rev_r = std::stod(tokens[0]);
                num_rev_l = std::stod(tokens[1]);
                ang_vel_r = std::stod(tokens[2]);
                ang_vel_l = std::stod(tokens[3]);
            }
            tokens.clear();
            
            // publish the wheel angular velocities
            ang_vels.data.clear();
            ang_vels.data.push_back(ang_vel_r);
            ang_vels.data.push_back(ang_vel_l);
            ang_vel_pub->publish(ang_vels);

            // calculate position based on wheel revolutions using differential drive equations
            dl = num_rev_r * wheel_circumference;
            dr = num_rev_l * wheel_circumference;
            d = (dl + dr) / 2;
            d_yaw = (dr - dl) / wheel_separation;
            
            odom.pose.pose.position.x += d * cos(yaw - (d_yaw / 2));
            odom.pose.pose.position.y += d * sin(yaw - (d_yaw / 2));
            
            yaw -= d_yaw;

            odom.header.stamp = node->now();
            imu.header.stamp = node->now();

            odom_pub->publish(odom);
            imu_pub->publish(imu);
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
