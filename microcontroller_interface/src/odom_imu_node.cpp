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

#define WHEEL_RADIUS 0.022
#define WHEEL_Y_OFF 0.048

const double WHEEL_CIRCUMFERENCE = 2.0 * M_PI * WHEEL_RADIUS;
const double TWO_WHEEL_Y_OFF = 2.0 * WHEEL_Y_OFF;

using namespace LibSerial;

constexpr const char* const SERIAL_PORT_1 = "/dev/ttyUSB0";

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("odom_imu_node");
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

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

    nav_msgs::msg::Odometry odom;
    sensor_msgs::msg::Imu imu;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    imu.header.frame_id = "base_footprint";

    double num_rev0;
    double num_rev1;
    double d;
    double dl;
    double dr;
    double d_yaw;
    double yaw = 0;
    while (rclcpp::ok()) 
    {
        try 
        {
            serial_port.ReadLine(line, '\n', timeout_ms); // read until the '\n' character
            std::stringstream ss(line);
            
            // print the line if one of the tokens is not a number
            if (!(ss >> imu.orientation.x &&
                  ss >> imu.orientation.y &&
                  ss >> imu.orientation.z &&
                  ss >> imu.orientation.w &&
                  ss >> num_rev0 &&
                  ss >> num_rev1))
            {
                std::cout << line;
                continue;
            }
            
            // calculate position based on wheel revolutions using differential drive equations
            dl = num_rev0 * WHEEL_CIRCUMFERENCE;
            dr = num_rev1 * WHEEL_CIRCUMFERENCE;
            d = (dl + dr) / 2;
            d_yaw = (dr - dl) / TWO_WHEEL_Y_OFF;
            
            odom.pose.pose.position.x += d * cos(yaw + (d_yaw / 2));
            odom.pose.pose.position.y += d * sin(yaw + (d_yaw / 2));
            
            yaw += d_yaw;

            odom.pose.pose.orientation.z *= sin(d_yaw / 2);
            odom.pose.pose.orientation.w *= cos(d_yaw / 2);

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