#pragma once
#ifndef ROS2WS_DEPTH_SUBSCRIBER_H
#define ROS2WS_DEPTH_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std::placeholders;

class DepthSubscriber : public rclcpp::Node
{
public:
    DepthSubscriber();
    ~DepthSubscriber();
private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void showDepthMap(cv::Mat map, uint32_t width , uint32_t height);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr FSubscriber;
};

#endif //ROS2WS_DEPTH_SUBSCRIBER_H
