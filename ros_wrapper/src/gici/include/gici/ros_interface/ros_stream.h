/**
 * @Function: Handle ROS stream publishing and subscribing
 *
 * @Author  : Cheng Chi
 * @Email   : chichengcn@sjtu.edu.cn
 *
 * Copyright (C) 2023 by Cheng Chi, All rights reserved.
 **/
#pragma once

#include <functional>
#include <glog/logging.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "gici/estimate/estimating.h"
#include "gici/ros_interface/ros_publisher.h"
#include "gici/stream/node_handle.h"

namespace gici
{

// ROS data format
enum class RosDataFormat
{
    Image,
    Imu,
    GnssRaw,
    PoseStamped,
    PoseWithCovarianceStamped,
    Odometry,
    NavSatFix,
    Marker,
    Path
};

// GNSS raw data format
enum class RosGnssDataFormat
{
    Observations,
    Ephemerides,
    AntennaPosition,
    IonosphereParameter,
    CodeBias,
    PhaseBias,
    EphemeridesCorrection
};

class RosStream : public Streaming
{
  public:
    using DataCallback = Streaming::DataCallback;
    using PipelineCallback = Streaming::PipelineConvert;

    RosStream(const RosNodePtr &node, const NodeOptionHandlePtr &nodes, int istreamer);
    ~RosStream();

    // Check if valid
    inline bool valid()
    {
        return valid_;
    }

    // Set estimator data callback
    void setDataCallback(const DataCallback &callback) override
    {
        data_callbacks_.push_back(callback);
    }

    // Output data callback
    void outputDataCallback(const std::string tag, const std::shared_ptr<DataCluster> &data) override;

    // Pipeline sends input data to logging stream
    void pipelineCallback(const std::string &tag, const std::shared_ptr<DataCluster> &data);

    // Get tag
    std::string getTag()
    {
        return tag_;
    }

    // Get I/O type
    StreamIOType getIoType()
    {
        return io_type_;
    }

    // Bind input and logging streams (ROS to ROS)
    static void bindLogWithInput();

    // Get instantiated objects
    static std::vector<RosStream *> &getObjects()
    {
        return static_this_;
    }

  private:
    // Send solution data to ROS topic
    void solutionOutputCallback(std::string tag, Solution &solution);

    // Send featured image to ROS topic
    void featuredImageOutputCallback(FramePtr &frame);

    // Send features as marker to ROS topic
    void mapPointOutputCallback(MapPtr &map);

    // Send GNSS raw data to ROS topics
    void gnssRawDataOutputCallback(DataCluster::GNSS &gnss);

    // Send IMU data to ROS topic
    void imuDataOutputCallback(DataCluster::IMU &imu);

    // Send image data to ROS topic
    void imageDataOutputCallback(DataCluster::Image &image);

    // ROS callbacks
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
    void gnssObservationsCallback(const gici_ros::GnssObservationsConstPtr &msg);
    void gnssEphemeridesCallback(const gici_ros::GnssEphemeridesConstPtr &msg);
    void gnssAntennaPositionCallback(const gici_ros::GnssAntennaPositionConstPtr &msg);
    void gnssIonosphereParameterCallback(const gici_ros::GnssIonosphereParameterConstPtr &msg);
    void gnssSsrCodeBiasesCallback(const gici_ros::GnssSsrCodeBiasesConstPtr &msg);
    void gnssSsrPhaseBiasesCallback(const gici_ros::GnssSsrPhaseBiasesConstPtr &msg);
    void gnssSsrEphemeridesCallback(const gici_ros::GnssSsrEphemeridesConstPtr &msg);
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg);
    void navSatFixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg);

    std::string resolveTopicName(const std::string &topic_name) const;

    template <typename MessageT>
    void addPublisher(const std::string &topic_name)
    {
        publishers_.emplace_back(node_->create_publisher<MessageT>(resolveTopicName(topic_name),
                                                                   rclcpp::QoS(queue_size_)));
    }

    template <typename MessageT, typename CallbackT>
    void addSubscription(const std::string &topic_name, CallbackT &&callback)
    {
        subscribers_.push_back(node_->create_subscription<MessageT>(resolveTopicName(topic_name),
                                                                    rclcpp::QoS(queue_size_),
                                                                    std::forward<CallbackT>(callback)));
    }

  protected:
    // Stream control
    std::string tag_;
    bool valid_;
    std::string input_ros_stream_tag_; // for ROS to ROS pipeline
    StreamIOType io_type_;
    std::vector<DataCallback> data_callbacks_; // call external function to send data out
    using PipelinesRosToRos = std::vector<PipelineCallback>;
    PipelinesRosToRos pipeline_ros_to_ros_; // sending data from ROS input to ROS log

    // ROS handles
    RosNodePtr node_;
    std::vector<RosPublisher> publishers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    std::vector<RosGnssDataFormat> gnss_formats_;
    std::unique_ptr<RosTransformBroadcaster> tranform_broadcaster_;
    GeoCoordinatePtr input_coordinate_;
    std::string frame_id_;
    std::string subframe_id_;
    std::string topic_name_;
    double marker_scale_ = 0.1;
    RosDataFormat data_format_;
    int queue_size_;
    std::unique_ptr<PathPublisher> path_publisher_;

    // Static variables for stream binding
    static std::vector<RosStream *> static_this_;
};

} // namespace gici
