//TODO: Change default values of zed parameters
#include "depth_subscriber.h"

uint8_t last_state_show = 0; // used to close opencv windows only one time.

DepthSubscriber::DepthSubscriber() : Node("depth_subscriber"), FBuffer(this->get_clock()), FListener(FBuffer)
{
    rclcpp::QoS depth_qos(10);
    depth_qos.keep_last(10);
    depth_qos.best_effort();
    depth_qos.durability_volatile();

    Node::declare_parameter<std::string>("topic_to_subscribe", "/point_cloud");
    Node::declare_parameter<float>("max_range", 3000.0);
    Node::declare_parameter<bool>("show_image", false);
    Node::declare_parameter<float>("sensor_properties.fx", 600);
    Node::declare_parameter<float>("sensor_properties.fy", 600);
    Node::declare_parameter<float>("sensor_properties.cx", 600);
    Node::declare_parameter<float>("sensor_properties.cy", 600);
    Node::declare_parameter<float>("sensor_height", 800);

    FSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->get_parameter("topic_to_subscribe").as_string(), depth_qos, std::bind(&DepthSubscriber::PointCloudCallback, this, _1));
    FMax_range = (float)this->get_parameter("max_range").as_double();
    Ffx = (float)this->get_parameter("sensor_properties.fx").as_double();
    Ffy = (float)this->get_parameter("sensor_properties.fy").as_double();
    Fcx = (float)this->get_parameter("sensor_properties.cx").as_double();
    Fcy = (float)this->get_parameter("sensor_properties.cy").as_double();
    FSensor_height = (float)this->get_parameter("sensor_height").as_double();

    FPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_filtered", 10);
    FPublisher_ground = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_ground", 10);
}

DepthSubscriber::~DepthSubscriber()
{
    cv::destroyAllWindows();
}
void DepthSubscriber::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received point cloud from %s with %d points. Height = %d %d %d %d %d\n", msg->header.frame_id.c_str(), msg->row_step*msg->height, msg->height, msg->width, msg->is_dense, msg->point_step, msg->is_bigendian);

    /* Get sensor orientation (roll, pitch , yaw) */
    geometry_msgs::msg::TransformStamped tf;
    tf = FBuffer.lookupTransform("map", "depth_sensor", rclcpp::Time(0));
    convertToEulerAngles(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);

    // Calculate the estimated ground plane equation
    Vector3D v1(0, std::cos(-FSensor_pitch), std::sin(-FSensor_pitch));
    Vector3D v2(std::cos(FSensor_roll), 0, std::sin(FSensor_roll));
    Vector3D abc = v1.cross(v2);
    Plane estimatedGround;
    estimatedGround.a = abc.x;
    estimatedGround.b = abc.y;
    estimatedGround.c = abc.z;
    estimatedGround.d = -FSensor_height * std::sqrt(estimatedGround.a * estimatedGround.a + estimatedGround.b * estimatedGround.b + estimatedGround.c * estimatedGround.c);

    //printf("%f %f %f %f\n", estimatedGround.a, estimatedGround.b, estimatedGround.c, estimatedGround.d);
    //printf("Vetores: 1: %f %f %f\t 2: %f %f %f\n", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);

    // TODO: DELETE
    int count = 0;
    auto msg_pc2 = sensor_msgs::msg::PointCloud2();

    // Read point cloud fields: x, y, z, rgba
    uint32_t total_bytes = msg->row_step * msg->height;
    for(int i = 0; i < total_bytes - 15;) {
        Point pt;
        union Float value_x, value_y, value_z;
        // Get x coordinate
        value_x.c[0] = msg->data[i];
        value_x.c[1] = msg->data[i+1];
        value_x.c[2] = msg->data[i+2];
        value_x.c[3] = msg->data[i+3];
        pt.x = value_x.f;
        // Get y coordinate
        value_y.c[0] = msg->data[i+4];
        value_y.c[1] = msg->data[i+5];
        value_y.c[2] = msg->data[i+6];
        value_y.c[3] = msg->data[i+7];
        pt.y = value_y.f;
        // Get z coordinate
        value_z.c[0] = msg->data[i+8];
        value_z.c[1] = msg->data[i+9];
        value_z.c[2] = msg->data[i+10];
        value_z.c[3] = msg->data[i+11];
        pt.z = value_z.f;
        // Get r, g, b, a values
        pt.r = msg->data[i+14];
        pt.g = msg->data[i+13];
        pt.b = msg->data[i+12];
        pt.a = ((float)msg->data[i+15] / 255);
        i += 16;

        // Calculate that distance of each point to the plane and consider only the points that are very close to the estimated plane
        auto norm = std::sqrt(estimatedGround.a * estimatedGround.a + estimatedGround.b * estimatedGround.b + estimatedGround.c * estimatedGround.c);
        if (std::fabs(estimatedGround.a * pt.x + estimatedGround.b * pt.y + estimatedGround.c * pt.z + estimatedGround.d) / norm < 200) {
            estimatedGround.points.push_back(pt);

            // TODO: DELETE
            for(int k = 0; k < 4; k++) {
                msg_pc2.data.push_back(value_x.c[k]);
            }
            for(int k = 0; k < 4; k++) {
                msg_pc2.data.push_back(value_y.c[k]);
            }
            for(int k = 0; k < 4; k++) {
                msg_pc2.data.push_back(value_z.c[k]);
            }
            msg_pc2.data.push_back(pt.b);
            msg_pc2.data.push_back(pt.g);
            msg_pc2.data.push_back(pt.r);
            msg_pc2.data.push_back(pt.a);
            count++;
        }
    }

    // TODO: Delete
    std::vector<sensor_msgs::msg::PointField> fields;
    auto field = sensor_msgs::msg::PointField();
    field.name = "x";
    field.datatype = 7;
    field.count = 1;
    field.offset = 0;
    fields.push_back(field);

    field.name = "y";
    field.datatype = 7;
    field.count = 1;
    field.offset = 4;
    fields.push_back(field);

    field.name = "z";
    field.datatype = 7;
    field.count = 1;
    field.offset = 8;
    fields.push_back(field);

    field.name = "rgb";
    field.datatype = 7;
    field.count = 1;
    field.offset = 12;
    fields.push_back(field);

    msg_pc2.header.frame_id = "map";
    msg_pc2.height = 1;
    msg_pc2.fields = fields;
    msg_pc2.is_bigendian = false;
    msg_pc2.point_step = 16;
    msg_pc2.is_dense = true;
    msg_pc2.row_step = msg_pc2.point_step * count;
    msg_pc2.width = count;
    if(count > 0)
        FPublisher->publish(msg_pc2);

    Plane final_ground_plane;
    RANSAC(estimatedGround, final_ground_plane);
    //printf("Nº de pontos no plano do chão: %lu\n", final_ground_plane.points.size());

    //TODO:Delete
    auto msg_pc_ground = sensor_msgs::msg::PointCloud2();
    std::vector<sensor_msgs::msg::PointField> fields_ground;
    auto field_ground = sensor_msgs::msg::PointField();
    field_ground.name = "x";
    field_ground.datatype = 7;
    field_ground.count = 1;
    field_ground.offset = 0;
    fields_ground.push_back(field_ground);

    field_ground.name = "y";
    field_ground.datatype = 7;
    field_ground.count = 1;
    field_ground.offset = 4;
    fields_ground.push_back(field_ground);

    field_ground.name = "z";
    field_ground.datatype = 7;
    field_ground.count = 1;
    field_ground.offset = 8;
    fields_ground.push_back(field_ground);

    field_ground.name = "rgb";
    field_ground.datatype = 7;
    field_ground.count = 1;
    field_ground.offset = 12;
    fields_ground.push_back(field_ground);
    uint32_t contador = final_ground_plane.points.size();
    msg_pc_ground.header.frame_id = "map";
    msg_pc_ground.height = 1;
    msg_pc_ground.fields = fields_ground;
    msg_pc_ground.is_bigendian = false;
    msg_pc_ground.point_step = 16;
    msg_pc_ground.is_dense = true;
    msg_pc_ground.row_step = msg_pc_ground.point_step * contador;
    msg_pc_ground.width = contador;

    union Float xg, yg, zg;
    for(int g = 0; g < contador; g++) {
        xg.f = final_ground_plane.points[g].x;
        yg.f = final_ground_plane.points[g].y;
        zg.f = final_ground_plane.points[g].z;
        for(int k = 0; k < 4; k++) {
            msg_pc_ground.data.push_back(xg.c[k]);
        }
        for(int k = 0; k < 4; k++) {
            msg_pc_ground.data.push_back(yg.c[k]);
        }
        for(int k = 0; k < 4; k++) {
            msg_pc_ground.data.push_back(zg.c[k]);
        }
        msg_pc_ground.data.push_back(final_ground_plane.points[g].b);
        msg_pc_ground.data.push_back(final_ground_plane.points[g].g);
        msg_pc_ground.data.push_back(final_ground_plane.points[g].r);
        msg_pc_ground.data.push_back(final_ground_plane.points[g].a);
    }
    FPublisher_ground->publish(msg_pc_ground);

    // Check if it is a danger situation
    uint32_t num_danger_points = 0;
    for(int i = 0; i < total_bytes - 15;) {
        Point pt;
        union Float value_x, value_y, value_z;
        // Get x coordinate
        value_x.c[0] = msg->data[i];
        value_x.c[1] = msg->data[i + 1];
        value_x.c[2] = msg->data[i + 2];
        value_x.c[3] = msg->data[i + 3];
        pt.x = value_x.f;
        if(pt.x < -400 || pt.x > 400) {
            i += 16;
            continue;
        }
        // Get y coordinate
        value_y.c[0] = msg->data[i + 4];
        value_y.c[1] = msg->data[i + 5];
        value_y.c[2] = msg->data[i + 6];
        value_y.c[3] = msg->data[i + 7];
        pt.y = value_y.f;
        // Get z coordinate
        value_z.c[0] = msg->data[i + 8];
        value_z.c[1] = msg->data[i + 9];
        value_z.c[2] = msg->data[i + 10];
        value_z.c[3] = msg->data[i + 11];
        pt.z = value_z.f;
        // Calculate the distance from the point to the plane
        auto norm = std::sqrt(final_ground_plane.a * final_ground_plane.a + final_ground_plane.b * final_ground_plane.b + final_ground_plane.c * final_ground_plane.c);
        if (std::fabs(final_ground_plane.a * pt.x + final_ground_plane.b * pt.y + final_ground_plane.c * pt.z + final_ground_plane.d) / norm > 60) {
            // Calculate the distance from the point to the sensor
            float distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if(distance < 740) {
                num_danger_points++;
            }
        }
        i += 16;
    }
    printf("Danger Points: %lu\n", num_danger_points);
    if(num_danger_points > 15000 && num_danger_points < 60000)
        printf("Possible danger! Be careful...\n");
    else if(num_danger_points >= 60000)
        printf("DANGER!!!\n");
}

void DepthSubscriber::convertToEulerAngles(const float qx, const float qy, const float qz, const float qw)
{
    // Consider that the received pitch is the roll and the received roll is the pitch...
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    FSensor_pitch = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        FSensor_roll = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        FSensor_roll = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    FSensor_yaw = std::atan2(siny_cosp, cosy_cosp);
    //printf("Roll: %f\tPitch: %f\tYaw: %f\n", FSensor_roll*180/3.14, FSensor_pitch*180/3.14, FSensor_yaw*180/3.14);
}



void DepthSubscriber::filterGroundPoints(const std::vector<Point>& in_pts, Plane& out_plane, const uint32_t width, const uint32_t height)
{
    for (int j = 0; j < width; )
    {
        for (int i = 0; i < height-5; )
        {
            int lower_idx = j + i * width;
            int upper_idx = j + (i + 5) * width;

            Point upper_pt = in_pts[upper_idx];
            Point lower_pt = in_pts[lower_idx];

            if (upper_pt.z == -1 || lower_pt.z == -1)
            {
                // no info to check, invalid points
                continue;
            }
            float dX = upper_pt.x - lower_pt.x;
            float dY = upper_pt.y - lower_pt.y;
            float dZ = upper_pt.z - lower_pt.z;

            //float vertical_angle = std::acos((dX * dX + dY * dY) / (std::sqrt(dX * dX + dY * dY) * std::sqrt(dX * dX + dY * dY + dZ * dZ)));
            float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));
            //printf("Vertical angle: %f\n", vertical_angle);
            if (abs(vertical_angle) <= 3*3.14156/180)
            {
                out_plane.points.push_back(lower_pt);
                out_plane.points.push_back(upper_pt);
            }
            i += 5;
        }
        j += 5;
    }
}

bool DepthSubscriber::RANSAC(const Plane& in_plane, Plane& final_ground_plane)
{
    int   max_idx       = in_plane.points.size();
    int   min_idx       = 0;
    int   max_tries     = 1000;
    int   c_max_inliers = 0;
    int max_iters       = 20;
    for (auto k = 0; k < max_iters; k++) {
        // Declare private point cloud to store current solution
        std::vector<Point> m_pcl;
        // Reset number of inliers in each iteration
        int num_inliers = 0;

        // Randomly select three points that cannot be cohincident
        bool found_valid_pts = false;
        int  n               = 0;
        int  idx1, idx2, idx3;
        while (!found_valid_pts) {
            idx1 = std::rand() % (max_idx - min_idx + 1) + min_idx;
            idx2 = std::rand() % (max_idx - min_idx + 1) + min_idx;
            idx3 = std::rand() % (max_idx - min_idx + 1) + min_idx;

            if (idx1 != idx2 && idx1 != idx3 && idx2 != idx3)
                found_valid_pts = true;

            n++;
            if (n > max_tries)
                break;
        }

        if (!found_valid_pts) {
            std::cout << "WARNING (ransac): No valid set of points found ... "
                      << std::endl;
            return false;
        }
        // Declare the 3 points selected on this iteration
        Point pt1 = Point(
                in_plane.points[idx1].x, in_plane.points[idx1].y, in_plane.points[idx1].z);
        Point pt2 = Point(
                in_plane.points[idx2].x, in_plane.points[idx2].y, in_plane.points[idx2].z);
        Point pt3 = Point(
                in_plane.points[idx3].x, in_plane.points[idx3].y, in_plane.points[idx3].z);
        // Extract the plane hessian coefficients
        Vector3D v1(pt2, pt1);
        Vector3D v2(pt3, pt1);
        Vector3D abc = v1.cross(v2);
        float    a   = abc.x;
        float    b   = abc.y;
        float    c   = abc.z;
        float    d   = -(a * pt1.x + b * pt1.y + c * pt1.z);

        for (const auto& m_pt : in_plane.points) {
            // Compute the distance each point to the plane - from
            // https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
            auto norm = std::sqrt(a * a + b * b + c * c);
            if (std::fabs(a * m_pt.x + b * m_pt.y + c * m_pt.z + d) / norm <
                30) {
                num_inliers++;
                m_pcl.push_back(m_pt);
            }
        }

        if (num_inliers > c_max_inliers) {
            c_max_inliers = num_inliers;

            final_ground_plane.a = a;
            final_ground_plane.b = b;
            final_ground_plane.c = c;
            final_ground_plane.d = d;
            final_ground_plane.points.clear();
            final_ground_plane.points = m_pcl;
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto depth_node = std::make_shared<DepthSubscriber>();

    rclcpp::spin(depth_node);

    rclcpp::shutdown();
    return 0;
}
