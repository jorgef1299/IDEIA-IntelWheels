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

#include <fstream>
#include <iostream>

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
    ~DepthSubscriber();
private:
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void showDepthMap(cv::Mat map, uint32_t width , uint32_t height);
    void convertPixel2World(const Point& in_pt, Point& out_pt);
    void convertToEulerAngles(const float qx, const float qy, const float qz, const float qw);
    void filterGroundPoints(const std::vector<Point>& in_pts, Plane& out_plane, const uint32_t width, const uint32_t height);
    bool RANSAC(const Plane& in_plane, Plane& final_ground_plane);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr FSubscriber;
    tf2_ros::Buffer FBuffer;
    tf2_ros::TransformListener FListener;
    float FMax_range;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> FRange_mat;
    float Ffx, Ffy;
    float Fcx, Fcy;
    float FSensor_height;  // Sensor height in relation to the ground //TODO: Adicionar parâmetro
    float FSensor_roll;
    float FSensor_pitch;
    float FSensor_yaw;
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

struct Pose {
    Pose()
    {
        x = 0;
        y = 0;
        z = 0;
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
    Pose(const float x_, const float y_, const float z_, const float roll_, const float pitch_, const float yaw_)
    {
        x = x_;
        y = y_;
        z = z_;
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
    }
    // Convert Euler angles to a Rotation matrix
    void toRotMatrix(std::array<float, 9>& m_rot) const
    {
        float ci = cos(roll);
        float cj = cos(pitch);
        float ch = cos(yaw);
        float si = sin(roll);
        float sj = sin(pitch);
        float sh = sin(yaw);
        float cc = ci * ch;
        float cs = ci * sh;
        float sc = si * ch;
        float ss = si * sh;

        // Store the values
        m_rot[0] = cj * ch;
        m_rot[1] = sj * sc - cs;
        m_rot[2] = sj * cc + ss;
        m_rot[3] = cj * sh;
        m_rot[4] = sj * ss + cc;
        m_rot[5] = sj * cs - sc;
        m_rot[6] = -sj;
        m_rot[7] = cj * si;
        m_rot[8] = cj * ci;
    }


    // Cartesian coordinates
    float x;
    float y;
    float z;
    // Euler angles
    float roll;
    float pitch;
    float yaw;
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
