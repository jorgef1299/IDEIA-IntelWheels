#pragma once
#ifndef ZED2_CAMERA_H
#define ZED2_CAMERA_H

#include <sl/Camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/image_encodings.hpp"
#include <csignal>
#include <sensor_msgs/msg/point_cloud2.hpp>

std::shared_ptr<sensor_msgs::msg::Image> imageToROSMsg(sl::Mat& img, std::string frameId, rclcpp::Time t);
rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type);

union Float {
        float    m_float;
        unsigned char  m_bytes[4];
};

class ZED2_camera : public rclcpp::Node
{
public:
    ZED2_camera();
    ~ZED2_camera();
    void getDepthMap();
    void publishDepthImage();
    void publishTF();
    void publishPointCloud();
private:
    void init_parameters();
    sl::Camera FZed;
    sl::InitParameters FInit_params;
    sl::Mat FDepth;
    sl::Pose FZed_pose;
    sl::Mat FPointCloud;
    image_transport::Publisher FPubDepth;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FPublisher;
};

#endif //ZED2_CAMERA_H