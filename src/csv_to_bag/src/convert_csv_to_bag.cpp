#include <fstream>
#include <iostream>
#include <istream>
#include <sstream>
#include <stdlib.h>
#include <streambuf>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

std::vector<std::string> path;

double scientificToDouble(const std::string &scientificNotation) {
    double value;
    std::istringstream iss(scientificNotation);
    iss >> value;
    return value;
}

void ReadData(const std::vector<std::string> &path, std::vector<double> &stamps,
              std::vector<Eigen::Vector3d> &accs,
              std::vector<Eigen::Vector3d> &gyros,
              std::vector<Eigen::Vector3d> &gpses,
              std::vector<Eigen::Vector3d> &ref_poses,
              std::vector<Eigen::Quaterniond> &ref_att_quats);

int main(int argc, char **argv) {
    ros::init(argc, argv, "csv_to_bag_node");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open("sim_imu.bag", 1U);

    path.push_back("time.csv");
    path.push_back("accel-0.csv");
    path.push_back("gyro-0.csv");
    path.push_back("gps-0.csv");
    path.push_back("ref_pos.csv");
    path.push_back("ref_att_quat.csv");

    std::vector<double> stamps;
    std::vector<Eigen::Vector3d> accs;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> gpses;
    std::vector<Eigen::Vector3d> ref_poses;
    std::vector<Eigen::Quaterniond> ref_att_quats;

    ReadData(path, stamps, accs, gyros, gpses, ref_poses, ref_att_quats);

    for (int i = 0; i < stamps.size(); i++) {
        if (ros::ok()) {
            ROS_DEBUG("the value size = %ld\n", stamps.size());
            sensor_msgs::Imu imu_msg;
            if (ros::Time(stamps[i]) < ros::TIME_MIN)
                continue;
            imu_msg.header.stamp = ros::Time(stamps[i]);
            imu_msg.linear_acceleration.x = accs[i].x();
            imu_msg.linear_acceleration.y = accs[i].y();
            imu_msg.linear_acceleration.z = accs[i].z();

            imu_msg.angular_velocity.x = gyros[i].x();
            imu_msg.angular_velocity.y = gyros[i].y();
            imu_msg.angular_velocity.z = gyros[i].z();
            bag.write("/imu", imu_msg.header.stamp, imu_msg);

            nav_msgs::Odometry odom;
            odom.child_frame_id = "ground_truth";
            odom.header.stamp = ros::Time(stamps[i]);
            odom.header.frame_id = "world";

            odom.pose.pose.position.x = ref_poses[i].x();
            odom.pose.pose.position.y = ref_poses[i].y();
            odom.pose.pose.position.z = ref_poses[i].z();

            odom.pose.pose.orientation.w = ref_att_quats[i].w();
            odom.pose.pose.orientation.x = ref_att_quats[i].x();
            odom.pose.pose.orientation.y = ref_att_quats[i].y();
            odom.pose.pose.orientation.z = ref_att_quats[i].z();
            bag.write("/odom", odom.header.stamp, odom);
        } else {
            break;
        }
    }
    return EXIT_SUCCESS;
}

void ReadData(const std::vector<std::string> &path, std::vector<double> &stamps,
              std::vector<Eigen::Vector3d> &accs,
              std::vector<Eigen::Vector3d> &gyros,
              std::vector<Eigen::Vector3d> &gpses,
              std::vector<Eigen::Vector3d> &ref_poses,
              std::vector<Eigen::Quaterniond> &ref_att_quats) {
    stamps.clear();
    accs.clear();
    gyros.clear();
    gpses.clear();
    ref_poses.clear();
    ref_att_quats.clear();

    std::vector<std::ifstream> reads;

    for (size_t i = 0; i < path.size(); i++) {
        reads.push_back(std::ifstream(path[i], std::ios::in));
    }

    bool init = false;
    while (true) {
        // 去除标题
        if (!init) {
            init = true;
            for (size_t i = 0; i < reads.size(); i++) {
                std::string strs;
                std::getline(reads[i], strs);
            }
        } else {
            double time;
            {
                if (reads[0].eof())
                    break;
                std::string strs;
                if (std::getline(reads[0], strs)) {
                    time = std::stod(strs);
                } else {
                    break;
                }
            }

            {
                std::string strs;
                std::string temp;
                strs = "";
                std::getline(reads[1], strs);
                temp = "";
                std::vector<double> acc;
                for (size_t i = 0; i < strs.size(); i++) {
                    if (strs[i] == ',') {
                        acc.push_back(std::stod(temp));
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                acc.push_back(std::stod(temp));

                strs = "";
                std::getline(reads[2], strs);
                temp = "";
                std::vector<double> gyro;
                for (size_t i = 0; i < strs.size(); i++) {
                    if (strs[i] == ',') {
                        gyro.push_back(std::stod(temp));
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                gyro.push_back(std::stod(temp));

                strs = "";
                std::getline(reads[3], strs);
                temp = "";
                std::vector<double> gps;
                for (int i = 0; i < strs.size(); ++i) {
                    if (strs[i] == ',') {
                        gps.push_back(std::stod(temp));
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                gps.push_back(std::stod(temp));

                strs = "";
                std::getline(reads[4], strs);
                temp = "";
                std::vector<double> ref_pos;
                for (int i = 0; i < strs.size(); ++i) {
                    if (strs[i] == ',') {
                        ref_pos.push_back(std::stod(temp));
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                ref_pos.push_back(std::stod(temp));

                strs = "";
                std::getline(reads[5], strs);
                temp = "";
                std::vector<double> ref_att_quat;
                for (int i = 0; i < strs.size(); ++i) {
                    if (strs[i] == ',') {
                        ref_att_quat.push_back(std::stod(temp));
                        temp = "";
                    } else {
                        temp = temp + strs[i];
                    }
                }
                ref_att_quat.push_back(std::stod(temp));

                stamps.push_back(time);
                accs.push_back(Eigen::Vector3d(acc[0], acc[1], acc[2]));
                gyros.push_back(Eigen::Vector3d(gyro[0], gyro[1], gyro[2]));

                Eigen::Quaterniond q =
                    Eigen::AngleAxisd(90, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(180, Eigen::Vector3d::UnitX());

                q = q.inverse();
                gpses.push_back(Eigen::Vector3d(gps[0], gps[1], gps[2]));

                ref_poses.push_back(
                    q * Eigen::Vector3d(ref_pos[0], ref_pos[1], ref_pos[2]));

                ref_att_quats.push_back(
                    q * Eigen::Quaterniond(ref_att_quat[0], ref_att_quat[1],
                                           ref_att_quat[2], ref_att_quat[3]));
            }
        }
    }
}