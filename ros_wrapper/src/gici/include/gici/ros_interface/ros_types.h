/**
 * @Function: Shared ROS2 types for the GICI ROS wrapper
 *
 * @Author  : Cheng Chi
 * @Email   : chichengcn@sjtu.edu.cn
 *
 * Copyright (C) 2023 by Cheng Chi, All rights reserved.
 **/
#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "gici_ros/msg/glonass_ephemeris.hpp"
#include "gici_ros/msg/gnss_antenna_position.hpp"
#include "gici_ros/msg/gnss_ephemerides.hpp"
#include "gici_ros/msg/gnss_ephemeris.hpp"
#include "gici_ros/msg/gnss_ionosphere_parameter.hpp"
#include "gici_ros/msg/gnss_observation.hpp"
#include "gici_ros/msg/gnss_observations.hpp"
#include "gici_ros/msg/gnss_ssr_code_bias.hpp"
#include "gici_ros/msg/gnss_ssr_code_biases.hpp"
#include "gici_ros/msg/gnss_ssr_ephemerides.hpp"
#include "gici_ros/msg/gnss_ssr_ephemeris.hpp"
#include "gici_ros/msg/gnss_ssr_phase_bias.hpp"
#include "gici_ros/msg/gnss_ssr_phase_biases.hpp"

namespace gici_ros
{
using GlonassEphemeris = msg::GlonassEphemeris;
using GnssAntennaPosition = msg::GnssAntennaPosition;
using GnssEphemerides = msg::GnssEphemerides;
using GnssEphemeris = msg::GnssEphemeris;
using GnssIonosphereParameter = msg::GnssIonosphereParameter;
using GnssObservation = msg::GnssObservation;
using GnssObservations = msg::GnssObservations;
using GnssSsrCodeBias = msg::GnssSsrCodeBias;
using GnssSsrCodeBiases = msg::GnssSsrCodeBiases;
using GnssSsrEphemerides = msg::GnssSsrEphemerides;
using GnssSsrEphemeris = msg::GnssSsrEphemeris;
using GnssSsrPhaseBias = msg::GnssSsrPhaseBias;
using GnssSsrPhaseBiases = msg::GnssSsrPhaseBiases;

using GnssAntennaPositionConstPtr = msg::GnssAntennaPosition::ConstSharedPtr;
using GnssEphemeridesConstPtr = msg::GnssEphemerides::ConstSharedPtr;
using GnssIonosphereParameterConstPtr = msg::GnssIonosphereParameter::ConstSharedPtr;
using GnssObservationsConstPtr = msg::GnssObservations::ConstSharedPtr;
using GnssSsrCodeBiasesConstPtr = msg::GnssSsrCodeBiases::ConstSharedPtr;
using GnssSsrEphemeridesConstPtr = msg::GnssSsrEphemerides::ConstSharedPtr;
using GnssSsrPhaseBiasesConstPtr = msg::GnssSsrPhaseBiases::ConstSharedPtr;
} // namespace gici_ros

namespace gici
{

using RosNode = rclcpp::Node;
using RosNodePtr = rclcpp::Node::SharedPtr;
using RosTime = rclcpp::Time;
using RosTransformBroadcaster = tf2_ros::TransformBroadcaster;

inline RosTime rosTimeFromSec(double seconds)
{
    return RosTime(static_cast<int64_t>(std::llround(seconds * 1.0e9)), RCL_ROS_TIME);
}

inline RosTime rosNow()
{
    return rclcpp::Clock(RCL_ROS_TIME).now();
}

class RosPublisher
{
  public:
    RosPublisher() = default;

    template <typename MessageT, typename AllocatorT>
    explicit RosPublisher(const std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> &publisher)
        : publisher_(publisher),
          publish_([publisher](const void *message) { publisher->publish(*static_cast<const MessageT *>(message)); })
    {
    }

    template <typename MessageT>
    void publish(const MessageT &message) const
    {
        if (!publish_)
        {
            throw std::runtime_error("ROS publisher is not initialized");
        }
        publish_(&message);
    }

    template <typename MessageT>
    void publish(const std::shared_ptr<MessageT> &message) const
    {
        publish(*message);
    }

  private:
    rclcpp::PublisherBase::SharedPtr publisher_;
    std::function<void(const void *)> publish_;
};

} // namespace gici
