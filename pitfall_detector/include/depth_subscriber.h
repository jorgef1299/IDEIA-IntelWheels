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
struct Quaternion;

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
    Quaternion findGroundNormal(float qx, float qy, float qz, float qw);
    bool RANSAC(const Plane& in_plane, Plane& final_ground_plane);
    void publishGroundPointCloud(Plane& ground_points);
    uint8_t evaluateDanger(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, uint32_t total_bytes, Plane& ground_plane, Quaternion sensor_orientation);
    float FMax_distance;
    float FSensor_height;  // Sensor height in relation to the ground
    float FSensor_normal_inclination;
    float FSensor_max_inclination;
    std::string FSensor_orientation;
    tf2_ros::Buffer FBuffer;  // tf buffer
    tf2_ros::TransformListener FListener;  // Listen to tf
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr FSubscriber;  // Subscriber for Point Cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FPublisher_ground;
    uint8_t count_errors_ransac;
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
    Vector3D cross(const Vector3D& other) const {
        Vector3D res;
        res.x = +(y * other.z - z * other.y);
        res.y = -(x * other.z - z * other.x);
        res.z = +(x * other.y - y * other.x);

        return res;
    }

    Vector3D operator+(const Vector3D& v) const
    {
        return Vector3D(x+v.x,y+v.y,z+v.z);
    }

    void operator*=(const float s)
    {
        x*=s;
        y*=s;
        z*=s;
    }

    Vector3D operator*(const float s) const
    {
        return Vector3D(s*x,s*y,s*z);
    }

    float operator*(const Vector3D& v)
    {
        return x*v.x+y*v.y+z*v.z;
    }

    float dot(const Vector3D& v) const{
        return x*v.x + y*v.y + z*v.z;
    }

    float length()
    {
        return std::sqrt(x*x + y*y + z *z);
    }
};

struct Quaternion
{
    float s;  // scalar
    Vector3D v;

    Quaternion(const float qw, const float qx, const float qy, const float qz)
    {
        s = qw;
        v.x = qx;
        v.y = qy;
        v.z = qz;
    }

    Quaternion(float s_, Vector3D v_)
    {
        s = s_;
        v.x = v_.x;
        v.y = v_.y;
        v.z = v_.z;
    }

    Quaternion operator+(const Quaternion& q)const
    {
        float scalar=s+q.s;
        Vector3D imaginary=v+q.v;

        return Quaternion(scalar,imaginary);
    }

    void operator*=(const Quaternion& q){

        (*this)=multiply(q);
    }

    void operator*=(float value){
        s*=value;
        v*=value;
    }

    Quaternion operator*(const Quaternion q)
    {
        float scalar=s*q.s - v.dot(q.v);
        Vector3D imaginary=q.v*s + v*q.s + v.cross(q.v);
        //printf("Operator*: %f %f %f\t %f %f %f\n", scalar, s*q.s, v.dot(q.v), imaginary.x, imaginary.y, imaginary.z);
        //printf("%f %f %f\t%f %f %f\n", v.x, v.y, v.z, q.v.x, q.v.y, q.v.z);
        return Quaternion(scalar,imaginary);
    }

    Quaternion multiply(const Quaternion& q) const
    {
        float scalar=s*q.s - v.dot(q.v);
        Vector3D imaginary=q.v*s + v*q.s + v.cross(q.v);
        return Quaternion(scalar,imaginary);
    }

    Quaternion operator*(const float value) const
    {
        float scalar=s*value;
        Vector3D imaginary=v*value;
        return Quaternion(scalar,imaginary);
    }
};

#endif //ROS2WS_DEPTH_SUBSCRIBER_H
