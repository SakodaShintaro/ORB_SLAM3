/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "../../../include/System.h"

#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

using namespace std;
using Image = sensor_msgs::msg::Image;
using Imu = sensor_msgs::msg::Imu;

double get_sec(const std_msgs::msg::Header & header)
{
  return header.stamp.sec + header.stamp.nanosec * 1e-9;
}

class MonoInertialNode : public rclcpp::Node
{
public:
  MonoInertialNode(const std::string & node_name, ORB_SLAM3::System * pSLAM)
  : Node(node_name), mpSLAM(pSLAM)
  {
    // Subscribe
    sub_image_ = create_subscription<Image>(
      "/image", rclcpp::QoS(rclcpp::KeepLast(100)),
      std::bind(&MonoInertialNode::callback_image, this, std::placeholders::_1));
    sub_imu_ = create_subscription<Imu>(
      "/imu/data", rclcpp::QoS(rclcpp::KeepLast(1000)),
      std::bind(&MonoInertialNode::callback_imu, this, std::placeholders::_1));

    // Publish pose
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
  }

  void callback_image(const Image::ConstSharedPtr & msg)
  {
    // Copy the rclcpp image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception & e) {
      // ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat img = cv_ptr->image.clone();
    const double time_of_image = get_sec(cv_ptr->header);

    // Load imu measurements from buffer
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    mBufMutex.lock();
    if (imuBuf.empty() || time_of_image > get_sec(imuBuf.back()->header)) {
      mBufMutex.unlock();
      return;
    }
    vImuMeas.clear();
    while (!imuBuf.empty() && get_sec(imuBuf.front()->header) <= time_of_image) {
      double t = get_sec(imuBuf.front()->header);
      cv::Point3f acc(
        imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y,
        imuBuf.front()->linear_acceleration.z);
      cv::Point3f gyr(
        imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y,
        imuBuf.front()->angular_velocity.z);
      vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
      imuBuf.pop();
    }
    mBufMutex.unlock();

    // mClahe->apply(img, img);

    // Exec SLAM
    const Sophus::SE3f pose = mpSLAM->TrackMonocular(img, time_of_image, vImuMeas);

    // Publish pose
    const Eigen::Quaternionf q(pose.rotationMatrix());
    geometry_msgs::msg::Pose result;
    result.position.x = pose.translation()[0];
    result.position.y = pose.translation()[1];
    result.position.z = pose.translation()[2];
    result.orientation.x = q.x();
    result.orientation.y = q.y();
    result.orientation.z = q.z();
    result.orientation.w = q.w();
    publish_pose(msg->header.stamp, result);
  }

  void callback_imu(const Imu::ConstSharedPtr & msg)
  {
    mBufMutex.lock();
    imuBuf.push(msg);
    mBufMutex.unlock();
  }

  void publish_pose(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
  {
    geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
    result_pose_stamped_msg.header.stamp = sensor_ros_time;
    result_pose_stamped_msg.header.frame_id = map_frame_;
    result_pose_stamped_msg.pose = result_pose_msg;
    pose_pub_->publish(result_pose_stamped_msg);
  }

  const std::string map_frame_ = "map";
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  ORB_SLAM3::System * mpSLAM;
  std::mutex mBufMutex;
  queue<Imu::ConstSharedPtr> imuBuf;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

  std::shared_ptr<MonoInertialNode> node = std::make_shared<MonoInertialNode>("Mono", &SLAM);

  rclcpp::spin(node);

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  rclcpp::shutdown();
}
