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

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

using namespace std;
using Image = sensor_msgs::msg::Image;

class ImageGrabber : public rclcpp::Node
{
public:
  ImageGrabber(const std::string & node_name, ORB_SLAM3::System * pSLAM)
  : Node(node_name), mpSLAM(pSLAM)
  {
    // Subscribe image
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    sub_ = create_subscription<Image>(
      "/image", qos, std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));

    // Publish pose
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
  }

  void GrabImage(const Image::ConstSharedPtr & msg)
  {
    // Copy the rclcpp image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception & e) {
      // ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    const double sec = cv_ptr->header.stamp.sec + cv_ptr->header.stamp.nanosec * 1e-9;
    const Sophus::SE3f pose = mpSLAM->TrackMonocular(cv_ptr->image, sec);
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
  rclcpp::Subscription<Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  ORB_SLAM3::System * mpSLAM;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

  std::shared_ptr<ImageGrabber> igb = std::make_shared<ImageGrabber>("Mono", &SLAM);

  rclcpp::spin(igb);

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  rclcpp::shutdown();
}
