/**
 * @Function: ROS publishers
 *
 * @Author  : Cheng Chi
 * @Email   : chichengcn@sjtu.edu.cn
 *
 * Copyright (C) 2023 by Cheng Chi, All rights reserved.
 **/
#pragma once

#include <iostream>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "gici/ros_interface/ros_types.h"
#include "gici/stream/formator.h"
#include "gici/utility/svo.h"

namespace gici
{

// Publish raw image
void publishImage(RosPublisher &pub, const cv::Mat &image, const RosTime time);
void publishImage(RosPublisher &pub, const FramePtr &frame, const RosTime time, const std::string &encoding);

// Publish image with features
void publishFeaturedImage(RosPublisher &pub, const FramePtr &frame, const RosTime time);

// Publish landmarks
void publishLandmarks(RosPublisher &pub, const MapPtr &map, const RosTime time, std::string frame_id,
                      double marker_scale = 0.1);

// Publish pose
void publishPoseStamped(RosPublisher &pub, const Transformation &pose, const RosTime time, std::string frame_id);

// Publish pose with covariance
void publishPoseWithCovarianceStamped(RosPublisher &pub, const Transformation &pose,
                                      const Eigen::Matrix<double, 6, 6> &covariance, const RosTime time,
                                      std::string frame_id);

// Publish pose with transform
void publishPoseWithTransform(RosPublisher &pub, RosTransformBroadcaster &broadcaster, const Transformation &pose,
                              const RosTime time, std::string frame_id, std::string child_frame_id);

// Publish pose with covariance and transform
void publishPoseWithCovarianceAndTransform(RosPublisher &pub, RosTransformBroadcaster &broadcaster,
                                           const Transformation &pose, const Eigen::Matrix<double, 6, 6> &covariance,
                                           const RosTime time, std::string frame_id, std::string child_frame_id);

// Publish odometry
void publishOdometry(RosPublisher &pub, RosTransformBroadcaster &broadcaster, const Transformation &pose,
                     const Eigen::Vector3d &velocity, const Eigen::Matrix<double, 9, 9> &covariance,
                     const RosTime time, std::string frame_id, std::string child_frame_id);

void publishNavSatFix(RosPublisher &pub, const Eigen::Vector3d &lla, Eigen::Matrix3d &covariance,
                      const RosTime time, GnssSolutionStatus status);

// Path publisher
class PathPublisher
{
  public:
    // Add a pose and publish all previous poses
    void addPoseAndPublish(RosPublisher &pub, const Transformation &pose, const RosTime time, std::string frame_id);

    // Clear previous poses
    inline void clear()
    {
        path_.poses.clear();
        is_initialized_ = false;
    }

  protected:
    bool is_initialized_ = false;
    nav_msgs::msg::Path path_;
};

// Publish 3D error
void publishError3d(RosPublisher &pub, const Eigen::Vector3d &error, const RosTime time, std::string frame_id);

// Publish IMU message
void publishImu(RosPublisher &pub, const DataCluster::IMU &imu);
void publishImu(RosPublisher &pub, const ImuMeasurement &imu);

// Publish GNSS message
void publishGnssObservations(RosPublisher &pub, const DataCluster::GNSS &gnss);
void publishGnssEphemerides(RosPublisher &pub, const DataCluster::GNSS &gnss);
void publishGnssAntennaPosition(RosPublisher &pub, const DataCluster::GNSS &gnss);
void publishGnssIonosphereParameter(RosPublisher &pub, const DataCluster::GNSS &gnss);
void publishGnssSsrCodeBiases(RosPublisher &pub, const DataCluster::GNSS &gnss);
void publishGnssSsrPhaseBiases(RosPublisher &pub, const DataCluster::GNSS &gnss);
void publishGnssSsrEphemerides(RosPublisher &pub, const DataCluster::GNSS &gnss);

} // namespace gici
