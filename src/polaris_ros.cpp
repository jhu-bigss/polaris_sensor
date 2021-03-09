// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <serial/serial.h>
#include <polaris_sensor/polaris_sensor.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <fstream>
#include <std_msgs/Float32.h>
#include <ctime>
#include <iostream>

#include <tf/transform_broadcaster.h>  // !!! add the tf .h

bool fexists(const std::string &filename) {
    std::ifstream ifile(filename.c_str());
    return (bool) ifile;
}

using namespace boost;
using namespace std;
using namespace polaris;


bool nexists(const std::string &r) {
    if (!fexists(r)) {
        ROS_WARN("Rom %s doest not exists, skipping.", r.c_str());
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
    ros::Time::init();
    ros::init(argc, argv, "polaris_sensor");
    ros::NodeHandle nh("~");

    ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("targets", 1);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("targets_cloud", 1);
    ros::Publisher dt_pub = nh.advertise<std_msgs::Float32>("dt", 1);

    tf::TransformBroadcaster broadcaster;  // !!!

    std::string port("/dev/ttyUSB0");
    if (!nh.getParam("port", port))
        ROS_WARN("Using default port: %s", port.c_str());
    else
        ROS_INFO("Using port: %s", port.c_str());

    std::string camera("polaris");
    if (!nh.getParam("camera", camera))
        ROS_WARN("Using default camera name: %s", camera.c_str());
    else
        ROS_INFO("Using camera name: %s", camera.c_str());

    double world_to_ndi_base_link_x;
    double world_to_ndi_base_link_y;
    double world_to_ndi_base_link_z;
    double world_to_ndi_base_link_rr;
    double world_to_ndi_base_link_rp;
    double world_to_ndi_base_link_ry;

    ros::NodeHandle nh_param(ros::this_node::getNamespace());
    nh_param.param("/world_to_ndi_base_link/x", world_to_ndi_base_link_x, 0.0);
    nh_param.param("/world_to_ndi_base_link/y", world_to_ndi_base_link_y, 0.0);
    nh_param.param("/world_to_ndi_base_link/z", world_to_ndi_base_link_z, 0.0);
    nh_param.param("/world_to_ndi_base_link/rr", world_to_ndi_base_link_rr, 0.0);
    nh_param.param("/world_to_ndi_base_link/rp", world_to_ndi_base_link_rp, 0.0);
    nh_param.param("/world_to_ndi_base_link/ry", world_to_ndi_base_link_ry, 0.0);

    tf::Transform transform_world_to_ndi_base_link;
    tf::Quaternion q_world_to_ndi_base_link;
    q_world_to_ndi_base_link.setRPY(
            world_to_ndi_base_link_rr,
            world_to_ndi_base_link_rp,
            world_to_ndi_base_link_ry);
    transform_world_to_ndi_base_link.setOrigin(tf::Vector3(
            world_to_ndi_base_link_x,
            world_to_ndi_base_link_y,
            world_to_ndi_base_link_z));
    transform_world_to_ndi_base_link.setRotation(q_world_to_ndi_base_link);

    std::vector<std::string> roms;
    std::string tmp;
    if (!nh.getParam("roms", tmp)) {
        ROS_FATAL("No rom provided, exiting.");
        return -1;
    }
    boost::erase_all(tmp, " ");
    boost::split(roms, tmp, boost::is_any_of(","));

    roms.erase(std::remove_if(roms.begin(), roms.end(), nexists),
               roms.end());
    if (roms.size() == 0) {
        ROS_FATAL("No roms could be loaded, exiting.");
        return -2;
    }
    int n = roms.size();
    Polaris polaris(port, roms);
    vector<std::string> ndi_marker_links(n);
    for (int i = 0; i < n; ++i) {
        ndi_marker_links[i] = "ndi_marker_" + getFileName(roms[i], "") + "_link";
    }

    geometry_msgs::PoseArray targets_pose;
    sensor_msgs::PointCloud targets_cloud;

    targets_cloud.header.frame_id = "/" + camera + "_link";
    targets_pose.header.frame_id = "/" + camera + "_link";

    ros::Rate loop_rate(100);
    int count = 0;
    ROS_INFO("Starting Polaris tracker loop");
    for (int i = 0; i < n; ++i) {
        targets_pose.poses.push_back(geometry_msgs::Pose());
        targets_cloud.points.push_back(geometry_msgs::Point32());
    }

    geometry_msgs::Pose pose;
    geometry_msgs::Point32 pt;

    std_msgs::Float32 dt;
    std::map<int, TransformationDataTX> targets;

    while (ros::ok()) {
        /* Start TX */
        std::string status;


        ros::Time start = ros::Time::now();

        polaris.readDataTX(status, targets);

        ros::Time end = ros::Time::now();
        ros::Duration duration = (end - start);

        dt.data = duration.nsec / 1000000.;

        dt_pub.publish(dt);

        std::map<int, TransformationDataTX>::iterator it = targets.begin();

        /* Start BX
        uint16_t status;
        std::map<int,TransformationDataBX> targets;
        polaris.readDataBX(status,targets);

        std::map<int,TransformationDataBX>::iterator it = targets.begin();*/

        broadcaster.sendTransform(
                tf::StampedTransform(
                        transform_world_to_ndi_base_link,
                        ros::Time::now(), 
                        "world",
                        "ndi_base_link"));  // !!! broadcast tf frame of ndi base link

        unsigned int i = 0;
        for (it = targets.begin(); it != targets.end(); ++it) {
            pose.position.x = it->second.tx;
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
            ROS_WARN_STREAM_DELAYED_THROTTLE(2.0, "RoM0 is not in the range");
        }

        targets_cloud.header.stamp = ros::Time::now();
        targets_pose.header.stamp = ros::Time::now();
        cloud_pub.publish(targets_cloud);
        pose_array_pub.publish(targets_pose);
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
            broadcaster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(
                                    tf::Quaternion(
                                            targets_pose.poses[i].orientation.x,
                                            targets_pose.poses[i].orientation.y,
                                            targets_pose.poses[i].orientation.z,
                                            targets_pose.poses[i].orientation.w),
                                    tf::Vector3(
                                            targets_pose.poses[i].position.x,
                                            targets_pose.poses[i].position.y,
                                            targets_pose.poses[i].position.z)),
                            ros::Time::now(),
                            "ndi_base_link",
                            ndi_marker_links[i]));
        }
//        broadcaster.sendTransform(
//                tf::StampedTransform(tf::Transform(
//                        tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
//                        tf::Vector3(pose.position.x, pose.position.y, pose.position.z)), ros::Time::now(),
//                                     "ndi_base_link", "ndi_marker")); // !!! broadcast the tf frame of marker

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
