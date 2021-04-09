#pragma once
#ifndef ZED2_CAMERA_H
#define ZED2_CAMERA_H

#include <sl/Camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include <csignal>

std::shared_ptr<sensor_msgs::msg::Image> imageToROSMsg(sl::Mat& img, std::string frameId, rclcpp::Time t);
rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type);

class ZED2_camera : public rclcpp::Node
{
public:
    ZED2_camera();
    ~ZED2_camera();
    void getDepthMap();
    void publishDepthImage();
private:
    void init_parameters();
    sl::Camera FZed;
    sl::InitParameters FInit_params;
    sl::Mat FDepth;
    image_transport::Publisher FPubDepth;
};

#endif //ZED2_CAMERA_H