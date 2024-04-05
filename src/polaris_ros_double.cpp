#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ctime>
#include <iostream>
#include <polaris_sensor/polaris_sensor.h>


bool fexists(const std::string &filename) {
    std::ifstream ifile(filename.c_str());
    return (bool) ifile;
}

using namespace std;
using namespace polaris;

string gen_random(const int len) {
    string tmp_s;
    static const char alphanum[] = "0123456789";

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
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("polaris_sensor");

    auto dt_pub = node->create_publisher<std_msgs::msg::Float32>("dt", 1);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    std::vector<std::string> roms;
    std::string tmp;
    node->declare_parameter("roms", std::string("/home/josh/BIGSS/ws_ndi_polaris/src/polaris_sensor/rom/LCSR-optical-tracker-body.rom"));
    if (!node->get_parameter("roms", tmp)) {
        RCLCPP_FATAL(node->get_logger(), "No rom provided, exiting.");
        return -1;
    }
    boost::erase_all(tmp, " ");
    boost::split(roms, tmp, boost::is_any_of(","));

    if (roms.empty()) {
        RCLCPP_FATAL(node->get_logger(), "No roms could be loaded, exiting.");
        return -2;
    }

    std::string port("/dev/ttyUSB0"); // Moved the port declaration here
    auto polaris = std::make_shared<Polaris>(port, roms);

    geometry_msgs::msg::Pose pose;

    std_msgs::msg::Float32 dt;
    std::map<int, TransformationDataTX> targets;

    rclcpp::Rate loop_rate(100);
    int count = 0;
    RCLCPP_INFO(node->get_logger(), "Starting Polaris tracker loop");

    while (rclcpp::ok()) {
        /* Start TX */
        std::string status;

        rclcpp::Time start = node->now();

        polaris->readDataTX(status, targets);

        rclcpp::Time end = node->now();
        rclcpp::Duration duration = (end - start);

        dt.data = duration.nanoseconds() / 1000000.;

        dt_pub->publish(dt);

        // Check if ndi_marker_1 and ndi_marker_2 are present
        if (targets.find(1) != targets.end() && targets.find(2) != targets.end()) {
            // Calculate relative transform between ndi_marker_1 and ndi_marker_2
            auto first_marker = targets.find(1)->second;
            auto second_marker = targets.find(2)->second;

            geometry_msgs::msg::TransformStamped relative_tf;
            relative_tf.header.stamp = node->now();
            relative_tf.header.frame_id = "ndi_marker_1_link";
            relative_tf.child_frame_id = "ndi_marker_2_link";

            // Calculate the relative translation
            relative_tf.transform.translation.x = second_marker.tx - first_marker.tx;
            relative_tf.transform.translation.y = second_marker.ty - first_marker.ty;
            relative_tf.transform.translation.z = second_marker.tz - first_marker.tz;

            // Calculate the relative rotation using quaternions
            tf2::Quaternion first_quaternion(
                first_marker.qx,
                first_marker.qy,
                first_marker.qz,
                first_marker.q0
            );

            tf2::Quaternion second_quaternion(
                second_marker.qx,
                second_marker.qy,
                second_marker.qz,
                second_marker.q0
            );

            // Compute the relative rotation by multiplying the inverse of the first quaternion with the second quaternion
            tf2::Quaternion relative_quaternion = first_quaternion.inverse() * second_quaternion;

            // Convert the relative quaternion to a ROS message format
            relative_tf.transform.rotation.x = relative_quaternion.getX();
            relative_tf.transform.rotation.y = relative_quaternion.getY();
            relative_tf.transform.rotation.z = relative_quaternion.getZ();
            relative_tf.transform.rotation.w = relative_quaternion.getW();

            // Publish the relative transform
            tf_broadcaster_->sendTransform(relative_tf);
        } else {
            RCLCPP_WARN_STREAM(node->get_logger(), "ndi_marker_1 or ndi_marker_2 is missing");
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
        ++count;
    }

    rclcpp::shutdown();

    return 0;
}
