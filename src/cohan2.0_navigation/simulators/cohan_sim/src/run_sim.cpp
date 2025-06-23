#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <rosgraph_msgs/Clock.h>
#include <yaml-cpp/yaml.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <dlfcn.h>

#define SIM_FPS 60
#define SIM_STEP (1.0 / SIM_FPS)

// Global variables
bool running = true;
ros::Publisher clock_pub;
std::vector<ros::Publisher> odom_pubs;
std::vector<ros::Publisher> scan_pubs;
std::vector<ros::Subscriber> cmd_vel_subs;
tf::TransformBroadcaster tf_broadcaster;

// Signal handler for clean shutdown
void signalHandler(int sig) {
    ROS_INFO("Received Ctrl+C, shutting down...");
    running = false;
}

// Callback for velocity commands
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, int robot_idx) {
    // Example: Set robot velocity in simulation library
    // lib.set_robot_velocity(robot_idx, msg->linear.x, msg->linear.y, msg->angular.z);
    ROS_INFO("Received velocity command for robot %d", robot_idx);
}

// Main simulation loop
void simulationLoop(void* lib_handle, const std::vector<std::string>& robot_names) {
    double sim_time = 0.0;
    ros::Rate rate(SIM_FPS);

    while (ros::ok() && running) {
        // Step simulation
        // Example: lib.step_simulation();
        sim_time += SIM_STEP;

        // Publish /clock
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = ros::Time(sim_time);
        clock_pub.publish(clock_msg);

        // Publish odometry and scan data for each robot
        for (size_t i = 0; i < robot_names.size(); ++i) {
            nav_msgs::Odometry odom_msg;
            sensor_msgs::LaserScan scan_msg;

            // Example: Fill odom_msg and scan_msg with data from simulation library

            odom_pubs[i].publish(odom_msg);
            scan_pubs[i].publish(scan_msg);

            // Broadcast TF
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(sim_time), "odom", "base_link"));
        }

        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cohan_sim");
    ros::NodeHandle nh;

    // Load simulation library
    void* lib_handle = dlopen("libsim.so", RTLD_LAZY);
    if (!lib_handle) {
        ROS_FATAL("Failed to load simulation library: %s", dlerror());
        return -1;
    }

    // Load configuration
    std::string config_path;
    nh.param("config_path", config_path, std::string("config.yaml"));
    YAML::Node config = YAML::LoadFile(config_path);

    std::vector<std::string> robot_names = config["robot_names"].as<std::vector<std::string>>();

    // Initialize publishers and subscribers
    clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
    for (size_t i = 0; i < robot_names.size(); ++i) {
        odom_pubs.push_back(nh.advertise<nav_msgs::Odometry>(robot_names[i] + "/odom", 10));
        scan_pubs.push_back(nh.advertise<sensor_msgs::LaserScan>(robot_names[i] + "/scan", 10));
        cmd_vel_subs.push_back(nh.subscribe<geometry_msgs::Twist>(robot_names[i] + "/cmd_vel", 10, boost::bind(cmdVelCallback, _1, i)));
    }

    // Register signal handler
    signal(SIGINT, signalHandler);

    // Start simulation loop
    simulationLoop(lib_handle, robot_names);

    // Cleanup
    dlclose(lib_handle);
    return 0;
}
