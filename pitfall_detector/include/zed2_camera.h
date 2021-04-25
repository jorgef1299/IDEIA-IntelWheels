#pragma once
#ifndef ZED2_CAMERA_H
#define ZED2_CAMERA_H

#include <sl/Camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

union Float {
        float    m_float;
        unsigned char  m_bytes[4];
};

class ZED2_camera : public rclcpp::Node
{
public:
    ZED2_camera();
    ~ZED2_camera();
    void getData();
    void publishTF();
    void publishPointCloud();
private:
    void init_parameters();
    sl::Camera FZed;
    sl::InitParameters FInit_params;
    sl::Pose FZed_pose;
    sl::Mat FPointCloud;
    float FMaxRange;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FPublisher;
};

#endif //ZED2_CAMERA_H