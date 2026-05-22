/**
 * @Function: Convert file from GICI IMU pack to rosbag
 *
 * @Author  : Cheng Chi
 * @Email   : chichengcn@sjtu.edu.cn
 *
 * Copyright (C) 2023 by Cheng Chi, All rights reserved.
 **/
#include "gici/gnss/gnss_common.h"
#include "gici/ros_utility/ros_types.h"
#include "gici/stream/format_image.h"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace gici;

const int image_width = 752;
const int image_height = 480;
const int image_step = 1;

const std::string topic_name = "/gici/image_raw";

int main(int argc, char **argv)
{
    char imagepack_path[1024];
    if (argc < 2)
    {
        return -1;
    }
    else if (argc == 2)
    {
        strcpy(imagepack_path, argv[1]);
    }

    if (image_step != 1)
    {
        std::cerr << "We only support image input with step size of 1!" << std::endl;
        return -1;
    }

    FILE *fp_imagepack = fopen(imagepack_path, "r");
    char buf[1034];
    sprintf(buf, "%s.bag", imagepack_path);
    rosbag2_cpp::Writer bag;
    bag.open(buf);

    int n = 0;
    img_t img;
    init_img(&img, image_width, image_height, image_step);
    while ((n = fread(buf, sizeof(char), 1034, fp_imagepack)) != 0)
    {
        for (int i = 0; i < n; i++)
        {
            if (!input_image(&img, buf[i]))
                continue;
            cv::Mat image_mat(img.height, img.width, CV_8UC(img.step), img.image);
            sensor_msgs::msg::Image::SharedPtr img_msg =
                cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, image_mat).toImageMsg();
            img_msg->header.stamp = rosTimeFromSec(gnss_common::gtimeToDouble(img.time));
            bag.write(*img_msg, topic_name, img_msg->header.stamp);
        }
    }

    fclose(fp_imagepack);
    free_img(&img);
    bag.close();

    return 0;
}
