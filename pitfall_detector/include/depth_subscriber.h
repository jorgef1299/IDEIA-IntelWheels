#pragma once
#ifndef ROS2WS_DEPTH_SUBSCRIBER_H
#define ROS2WS_DEPTH_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <eigen3/Eigen/Dense>
// Transform headers
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.h>
// Point Cloud header
#include "sensor_msgs/msg/point_cloud2.hpp"


using namespace std::placeholders;

class DepthSubscriber;
struct Point;
struct Pose;
struct Plane;
struct Vector3D;

union Float {
    float f;
    unsigned char c[4];
};


class DepthSubscriber : public rclcpp::Node
{
public:
    DepthSubscriber();
private:
    void PointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void convertQuaternionsToEulerAngles(float qx, float qy, float qz, float qw);
    bool RANSAC(const Plane& in_plane, Plane& final_ground_plane);
    float FMax_distance;
    float FSensor_height;  // Sensor height in relation to the ground
    std::string FSensor_orientation;
    float FSensor_roll;
    float FSensor_pitch;
    tf2_ros::Buffer FBuffer;  // tf buffer
    tf2_ros::TransformListener FListener;  // Listen to tf
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr FSubscriber;  // Subscriber for Point Cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FPublisher_ground;
};

struct Point {
    Point() {
        x = 0;
        y = 0;
        z = 0;
    }
    Point(const float x_, const float y_, const float z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }
    Point operator=(const Point& new_point)
    {
        x = new_point.x;
        y = new_point.y;
        z = new_point.z;

        return *this;
    }
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    float a;
};

struct Plane
{
    std::vector<Point> points;
    float a;
    float b;
    float c;
    float d;
};

struct Vector3D : public Point
{
    Vector3D() = default;

    // A vector from three components
    Vector3D(const float& a, const float& b, const float& c)
    {
        x = a;
        y = b;
        z = c;
    }

    // A vector resulting from two points: v = p2 - p1
    Vector3D(Point pt2, Point pt1)
    {
        x = pt2.x - pt1.x;
        y = pt2.y - pt1.y;
        z = pt2.z - pt1.z;
    }

    // Cross product between two vectors
    Vector3D cross(const Vector3D& other)
    {
        Vector3D res;
        res.x = +(y * other.z - z * other.y);
        res.y = -(x * other.z - z * other.x);
        res.z = +(x * other.y - y * other.x);

        return res;
    }

};
#endif //ROS2WS_DEPTH_SUBSCRIBER_H
