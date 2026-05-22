/**
 * @Function: Publish poses
 *
 * @Author  : Cheng Chi
 * @Email   : chichengcn@sjtu.edu.cn
 *
 * Copyright (C) 2023 by Cheng Chi, All rights reserved.
 **/
#include "gici/ros_utility/ros_publisher.h"

using namespace gici;

int main(int argc, char **argv)
{
    // Initialize ROS
    std::vector<std::string> arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = RosNode::make_shared("gici");

    // Get file
    if (arguments.size() != 4)
    {
        std::cerr << "Invalid input variables! Supported variables are: "
                  << "<path-to-executable> <path-to-file> <topic-name> <time-duration>" << std::endl;
        rclcpp::shutdown();
        return -1;
    }
    std::string file_path = arguments[1];
    std::string topic_name = arguments[2];
    double duration = atof(arguments[3].data());

    RosPublisher path_pub(node->create_publisher<nav_msgs::msg::Path>("/" + topic_name + "/path", rclcpp::QoS(10)));
    RosPublisher pose_pub(node->create_publisher<nav_msgs::msg::Odometry>("/" + topic_name + "/pose", rclcpp::QoS(3)));

    std::unique_ptr<RosTransformBroadcaster> tranform_broadcaster_ =
        std::make_unique<RosTransformBroadcaster>(node);
    std::unique_ptr<PathPublisher> path_publisher_ = std::make_unique<PathPublisher>();

    char buf[1024];
    FILE *fp_tum = fopen(file_path.data(), "r");

    rclcpp::Rate r(1.0 / duration);
    while (rclcpp::ok() && !(fgets(buf, 1024 * sizeof(char), fp_tum) == NULL))
    {
        if (buf[0] == '#')
            continue;
        double time, position[3], quaternion[4];
        sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &position[0], &position[1], &position[2], &quaternion[0],
               &quaternion[1], &quaternion[2], &quaternion[3]);
        Eigen::Quaterniond q(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
        Transformation T_WS(Eigen::Vector3d(position), q.normalized());

        path_publisher_->addPoseAndPublish(path_pub, T_WS, rosTimeFromSec(time), "World");
        publishOdometry(pose_pub, *tranform_broadcaster_, T_WS, Eigen::Vector3d::Zero(),
                        Eigen::Matrix<double, 9, 9>::Zero(), rosTimeFromSec(time), "World", "Body");

        r.sleep();
    }

    fclose(fp_tum);
    rclcpp::shutdown();

    return 0;
}
