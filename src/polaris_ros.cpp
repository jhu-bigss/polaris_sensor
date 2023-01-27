// ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "polaris_sensor/polaris_sensor.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <fstream>
#include "std_msgs/msg/float32.hpp"
#include <ctime>
#include <iostream>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/parameter.hpp"

bool fexists(const std::string &filename) {
    std::ifstream ifile(filename.c_str());
    return (bool) ifile;
}

using namespace boost;
using namespace std;
using namespace polaris;


bool nexists(const std::string &r) {
    if (!fexists(r)) {
        // RCLCPP_WARN(node->get_logger(), "Rom %s doest not exists, skipping.",r.c_str());
        return true;
    }
    return false;
}

string gen_random(const int len) {
    string tmp_s;
    static const char alphanum[] =
            "0123456789";
//            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
//            "abcdefghijklmnopqrstuvwxyz";

    srand((unsigned) time(NULL) * getpid());

    tmp_s.reserve(len);

    for (int i = 0; i < len; ++i)
        tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
    return tmp_s;
}

string getFileName(const string &s, const string &default_val) {

    char sep = '/';
    char dot = '.';

    size_t i = s.rfind(sep, s.length());
    if (i != string::npos) {
        string sub_s = s.substr(i + 1, s.length() - i);
        size_t sub_i = sub_s.find(dot, 0);
        if (sub_i != 0) {
            return (sub_s.substr(0, sub_i));
        }
    }

    return (default_val + gen_random(8));
}

int main(int argc, char **argv) {
    // Usage : rosrun polaris_sensor polaris_sensor _roms:="/home/T0.rom,/home/T1.rom" _port:=/dev/ttyUSB0
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("polaris_sensor");

    auto pose_array_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("targets", 1);
    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud>("targets_cloud", 1);
    auto dt_pub = node->create_publisher<std_msgs::msg::Float32>("dt", 1);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    std::string port("/dev/ttyUSB0");
    node->declare_parameter("port", port);
    if (!node->get_parameter("port", port))
        RCLCPP_WARN(node->get_logger(), "Using default port: %s", port.c_str());
    else
        RCLCPP_INFO(node->get_logger(), "Using port: %s", port.c_str());

    std::string camera("polaris");
    node->declare_parameter("camera", camera);
    if (!node->get_parameter("camera", camera))
        RCLCPP_WARN(node->get_logger(), "Using default camera name: %s", camera.c_str());
    else
        RCLCPP_INFO(node->get_logger(), "Using camera name: %s", camera.c_str());

    double world_to_ndi_base_link_x;
    double world_to_ndi_base_link_y;
    double world_to_ndi_base_link_z;
    double world_to_ndi_base_link_rx;
    double world_to_ndi_base_link_ry;
    double world_to_ndi_base_link_rz;
    double world_to_ndi_base_link_rw;

    node->get_parameter_or<double>("/world_to_ndi_base_link/x", world_to_ndi_base_link_x, 0.0);
    node->get_parameter_or<double>("/world_to_ndi_base_link/y", world_to_ndi_base_link_y, 0.0);
    node->get_parameter_or<double>("/world_to_ndi_base_link/z", world_to_ndi_base_link_z, 0.0);
    node->get_parameter_or<double>("/world_to_ndi_base_link/rx", world_to_ndi_base_link_rx, 0.0);
    node->get_parameter_or<double>("/world_to_ndi_base_link/ry", world_to_ndi_base_link_ry, 0.0);
    node->get_parameter_or<double>("/world_to_ndi_base_link/rz", world_to_ndi_base_link_rz, 0.0);
    node->get_parameter_or<double>("/world_to_ndi_base_link/rw", world_to_ndi_base_link_rw, 0.0);

    std::vector<std::string> roms;
    std::string tmp;
    node->declare_parameter("roms", std::string("/home/lucy/moveit2_ws/NDI-serial-ros2/src/polaris_sensor/rom/LCSR-optical-tracker-body.rom"));
    if (!node->get_parameter("roms", tmp)) {
        RCLCPP_FATAL(node->get_logger(), "No rom provided, exiting.");
        return -1;
    }
    boost::erase_all(tmp, " ");
    boost::split(roms, tmp, boost::is_any_of(","));

    roms.erase(std::remove_if(roms.begin(), roms.end(), nexists),
               roms.end());
    if (roms.size() == 0) {
        RCLCPP_FATAL(node->get_logger(), "No roms could be loaded, exiting.");
        return -2;
    }
    int n = roms.size();
    Polaris polaris(port, roms);
    vector<std::string> ndi_marker_links(n);
    for (int i = 0; i < n; ++i) {
        ndi_marker_links[i] = "ndi_marker_" + getFileName(roms[i], "") + "_link";
    }

    geometry_msgs::msg::PoseArray targets_pose;
    sensor_msgs::msg::PointCloud targets_cloud;

    targets_cloud.header.frame_id = "/" + camera + "_link";
    targets_pose.header.frame_id = "/" + camera + "_link";

    rclcpp::Rate loop_rate(100);
    int count = 0;
    RCLCPP_INFO(node->get_logger(), "Starting Polaris tracker loop");
    for (int i = 0; i < n; ++i) {
        targets_pose.poses.push_back(geometry_msgs::msg::Pose());
        targets_cloud.points.push_back(geometry_msgs::msg::Point32());
    }

    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Point32 pt;

    std_msgs::msg::Float32 dt;
    std::map<int, TransformationDataTX> targets;

    while (rclcpp::ok()) {
        /* Start TX */
        std::string status;

        rclcpp::Time start = node->now();

        polaris.readDataTX(status, targets);

        rclcpp::Time end = node->now();
        rclcpp::Duration duration = (end - start);

        dt.data = duration.nanoseconds() / 1000000.;

        dt_pub->publish(dt);

        std::map<int, TransformationDataTX>::iterator it = targets.begin();

        /* Start BX
        uint16_t status;
        std::map<int,TransformationDataBX> targets;
        polaris.readDataBX(status,targets);

        std::map<int,TransformationDataBX>::iterator it = targets.begin();*/
        geometry_msgs::msg::TransformStamped tf_static_world_to_ndi_base;
        tf_static_world_to_ndi_base.header.stamp = node->now();
        tf_static_world_to_ndi_base.header.frame_id = "ndi_base_link";
        tf_static_world_to_ndi_base.child_frame_id = "world";
        tf_static_world_to_ndi_base.transform.translation.x = world_to_ndi_base_link_x;
        tf_static_world_to_ndi_base.transform.translation.y = world_to_ndi_base_link_y;
        tf_static_world_to_ndi_base.transform.translation.z = world_to_ndi_base_link_z;
        tf_static_world_to_ndi_base.transform.rotation.x = world_to_ndi_base_link_rx;
        tf_static_world_to_ndi_base.transform.rotation.y = world_to_ndi_base_link_ry;
        tf_static_world_to_ndi_base.transform.rotation.z = world_to_ndi_base_link_rz;
        tf_static_world_to_ndi_base.transform.rotation.w = world_to_ndi_base_link_rw;

        tf_broadcaster_->sendTransform(tf_static_world_to_ndi_base);

        unsigned int i = 0;
        for (it = targets.begin(); it != targets.end(); ++it) {
            pose.position.x = it->second.tx;
            RCLCPP_WARN_STREAM(node->get_logger(), "out put for target index " << it->second.error);
            RCLCPP_WARN_STREAM(node->get_logger(), "out put for target status " << status);
            RCLCPP_WARN_STREAM(node->get_logger(), "out put for target x " << pose.position.x);
            pose.position.y = it->second.ty;
            pose.position.z = it->second.tz;
            pose.orientation.x = it->second.qx;
            pose.orientation.y = it->second.qy;
            pose.orientation.z = it->second.qz;
            pose.orientation.w = it->second.q0;
            targets_pose.poses[i] = pose;

            pt.x = it->second.tx;
            pt.y = it->second.ty;
            pt.z = it->second.tz;
            targets_cloud.points[i] = pt;
            i++;
        }
        if (isnan(targets_pose.poses[0].position.x)) {
            RCLCPP_WARN_STREAM(node->get_logger(), "RoM0 is not in the range");
        }

        targets_cloud.header.stamp = node->now();
        targets_pose.header.stamp = node->now();
        cloud_pub->publish(targets_cloud);
        pose_array_pub->publish(targets_pose);
        for (int i = 0; i < n; ++i) {
            if (isnan(targets_pose.poses[i].position.x) ||
                isnan(targets_pose.poses[i].position.y) ||
                isnan(targets_pose.poses[i].position.z) ||
                isnan(targets_pose.poses[i].orientation.x) ||
                isnan(targets_pose.poses[i].orientation.y) ||
                isnan(targets_pose.poses[i].orientation.z) ||
                isnan(targets_pose.poses[i].orientation.w)) {
                targets_pose.poses[i].position.x = 0.0;
                targets_pose.poses[i].position.y = 0.0;
                targets_pose.poses[i].position.z = 0.0;
                targets_pose.poses[i].orientation.x = 0.0;
                targets_pose.poses[i].orientation.y = 0.0;
                targets_pose.poses[i].orientation.z = 0.0;
                targets_pose.poses[i].orientation.w = 1.0;
            }
            
            geometry_msgs::msg::TransformStamped tf_ndi_marker;
            
            tf_ndi_marker.header.stamp = node->now();
            tf_ndi_marker.header.frame_id = "ndi_base_link";
            tf_ndi_marker.child_frame_id = ndi_marker_links[i];

            tf_ndi_marker.transform.translation.x = targets_pose.poses[i].position.x;
            tf_ndi_marker.transform.translation.y = targets_pose.poses[i].position.y;
            tf_ndi_marker.transform.translation.z = targets_pose.poses[i].position.z;

            tf_ndi_marker.transform.rotation.x = targets_pose.poses[i].orientation.x;
            tf_ndi_marker.transform.rotation.y = targets_pose.poses[i].orientation.y;
            tf_ndi_marker.transform.rotation.z = targets_pose.poses[i].orientation.z;
            tf_ndi_marker.transform.rotation.w = targets_pose.poses[i].orientation.w;

            tf_broadcaster_->sendTransform(tf_ndi_marker);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
