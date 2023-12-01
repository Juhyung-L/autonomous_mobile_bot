#include <memory>
#include <chrono>
#include <lgpio.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

// left wheel PWM pin
#define ENA 6
// right wheel PWM pin
#define ENB 26
// left wheel direction pins
#define AIN1 12
#define AIN2 13
// right wheel direction pins
#define BIN1 20
#define BIN2 21

#define PWM_FREQ 100

#define CHECK_HANDLE(handle) \
    do { \
        if (handle < 0) { \
            std::cerr << "Bad handle at " << __FILE__ << ":" << __LINE__ << std::endl; \
            return -1; \
        } \
    } while(false) \


using namespace std::placeholders;

// physical characterisitics of robot
double wheel_radius, wheel_separation;
// tell compiler these variables can change at any time
volatile double wheel_vel_r = 0;
volatile double wheel_vel_l = 0;
// discrete integration term for PID
double e_int_r = 0;
double e_int_l = 0;
// previous error needed to calculate de/dt for PID
double prev_e_r = 0;
double prev_e_l = 0;
// PID parameters
double kp, kd, ki;
double max_duration; // maximum duration between curr_time and prev_time allowed for PID
// pwm values
double pwm_r = 0;
double pwm_l = 0;

int h; // handle for lgpio library

std::chrono::_V2::high_resolution_clock::time_point prev_time = std::chrono::high_resolution_clock::now();

void cleanUp()
{
    lgGpioWrite(h, AIN1, 0);
    lgGpioWrite(h, AIN2, 0);
    lgGpioWrite(h, BIN1, 0);
    lgGpioWrite(h, BIN2, 0);
    lgGpioWrite(h, ENA, 0);
    lgGpioWrite(h, ENB, 0);

    lgGpioFree(h, AIN1);
    lgGpioFree(h, AIN2);
    lgGpioFree(h, BIN1);
    lgGpioFree(h, BIN2);
    lgGpioFree(h, ENA);
    lgGpioFree(h, ENB);
 
    lgGpiochipClose(h);
}

double computeGain(double dt, double e, double& e_int, double& prev_e)
{
    e_int += e * dt;
    double gain = (kp * e) + (kd * (e - prev_e) / dt) + (ki * e_int);
    prev_e = e;
    return gain;
}

void wheelVelsCallback(const std_msgs::msg::Float64MultiArray& msg)
{
    wheel_vel_r = msg.data[0];
    wheel_vel_l = msg.data[1];
}

void cmdVelCallback(const geometry_msgs::msg::Twist& msg) // PID control happens in this function
{
    auto curr_time = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(curr_time - prev_time).count();
    prev_time = curr_time;
    if (dt > max_duration)
    {
        e_int_r = 0;
        e_int_l = 0;
        prev_e_r = 0;
        prev_e_l = 0;
        return;
    }

    // convert robot velocity to individual wheel velocities
    double target_wheel_vel_r = ((msg.linear.x * 2 / wheel_radius) + (msg.angular.z * wheel_separation / wheel_radius)) / 2;
    double target_wheel_vel_l = (msg.linear.x * 2 / wheel_radius) - target_wheel_vel_r;

    double e_r = target_wheel_vel_r - wheel_vel_r;
    double e_l = target_wheel_vel_l - wheel_vel_l;

    pwm_r += computeGain(dt, e_r, e_int_r, prev_e_r);
    pwm_l += computeGain(dt, e_l, e_int_l, prev_e_l);
    
    // cap to 100 because max PWM duty cycle is 100%
    if (pwm_r > 100.0)
    {
        pwm_r = 100.0;
    }
    else if (pwm_r < -100.0)
    {
        pwm_r = -100.0;
    }

    if (pwm_l > 100.0)
    {
        pwm_l = 100.0;
    }
    else if (pwm_l < -100.0)
    {
        pwm_l = -100.0;
    }
    
    // set the direction and PWM duty cycle
    if (pwm_r < 0)
    {
        lgGpioWrite(h, BIN1, 1);
        lgGpioWrite(h, BIN2, 0);
    }
    else
    {
        lgGpioWrite(h, BIN1, 0);
        lgGpioWrite(h, BIN2, 1);
    }
    if (pwm_l < 0)
    {
        lgGpioWrite(h, AIN1, 1);
        lgGpioWrite(h, AIN2, 0);
    }
    else
    {
        lgGpioWrite(h, AIN1, 0);
        lgGpioWrite(h, AIN2, 1);
    }

    lgTxPwm(h, ENB, PWM_FREQ, abs(pwm_r), 0, 0);
    lgTxPwm(h, ENA, PWM_FREQ, abs(pwm_l), 0, 0);
}

int main(int argc, char* argv[])
{
    int chip = 0;
    h = lgGpiochipOpen(chip);
    CHECK_HANDLE(h);

    // claim gpios
    lgGpioClaimOutput(h, 0, AIN1, 0);
    lgGpioClaimOutput(h, 0, AIN2, 0);
    lgGpioClaimOutput(h, 0, BIN1, 0);
    lgGpioClaimOutput(h, 0, BIN2, 0);
    lgGpioClaimOutput(h, 0, ENA, 0);
    lgGpioClaimOutput(h, 0, ENB, 0);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("motor_control_node");

    node->declare_parameter<double>("wheel_radius", 0.022);
    node->declare_parameter<double>("wheel_separation", 0.097);
    node->declare_parameter<double>("kp", 0.01);
    node->declare_parameter<double>("kd", 0.01);
    node->declare_parameter<double>("ki", 0.01);
    node->declare_parameter<double>("max_duration", 1.0);

    wheel_radius = node->get_parameter("wheel_radius").as_double();
    wheel_separation = node->get_parameter("wheel_separation").as_double();
    kp = node->get_parameter("kp").as_double();
    kd = node->get_parameter("kd").as_double();
    ki = node->get_parameter("ki").as_double();
    max_duration = node->get_parameter("max_duration").as_double();

    rclcpp::SubscriptionOptions wheel_vels_listener_options;
    wheel_vels_listener_options.callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto wheel_vels_listener = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "wheel_vels", rclcpp::SensorDataQoS(), std::bind(&wheelVelsCallback, _1), wheel_vels_listener_options
    );

    rclcpp::SubscriptionOptions cmd_vel_listener_options;
    cmd_vel_listener_options.callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto cmd_vel_listener = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SensorDataQoS(), std::bind(&cmdVelCallback, _1), cmd_vel_listener_options
    );

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    cleanUp();
    rclcpp::shutdown();
    
    return 0;
}
