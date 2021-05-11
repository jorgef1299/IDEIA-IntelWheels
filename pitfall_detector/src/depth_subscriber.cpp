#include "depth_subscriber.h"

DepthSubscriber::DepthSubscriber() : Node("depth_subscriber"), FBuffer(this->get_clock()), FListener(FBuffer)
{
    rclcpp::QoS depth_qos(10);
    depth_qos.keep_last(10);
    depth_qos.best_effort();
    depth_qos.durability_volatile();

    Node::declare_parameter<std::string>("topic_to_subscribe", "/point_cloud");
    Node::declare_parameter<float>("max_distance", 740.0);  // max distance to be danger
    Node::declare_parameter<float>("sensor_height", 800.0);  // distance from the ground to the sensor in the z axis
    Node::declare_parameter<uint8_t>("initial_inclination", 30);  // normal inclination of the sensor
    Node::declare_parameter<uint8_t>("max_relative_inclination", 30);  // maximum inclination relative to the initial inclination
    Node::declare_parameter<std::string>("orientation", "horizontal");
    Node::declare_parameter<bool>("publish_ground_point_cloud", false);
    Node::declare_parameter("evaluate_danger.minimum_points_level1", 10000);
    Node::declare_parameter("evaluate_danger.minimum_points_level2", 40000);

    FSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->get_parameter("topic_to_subscribe").as_string(), depth_qos, std::bind(&DepthSubscriber::PointCloudCallback, this, _1));
    FMax_distance = (float)this->get_parameter("max_distance").as_double();
    FSensor_height = (float)this->get_parameter("sensor_height").as_double();
    FSensor_normal_inclination = (float)this->get_parameter("initial_inclination").as_int() * M_PI / 180;  // Initial inclination of the sensor in radians
    FSensor_max_inclination = (float)this->get_parameter("max_relative_inclination").as_int() * M_PI / 180;  // Maximum inclination relative to the initial, in radians
    if(FSensor_normal_inclination < 0 || FSensor_normal_inclination > M_PI/2 || FSensor_max_inclination < 10*M_PI/180 || FSensor_max_inclination > M_PI/2) {
        std::cout << "Error on inclination parameters..." << std::endl;
        exit(EXIT_FAILURE);
    }
    FSensor_orientation = this->get_parameter("orientation").as_string();
    if(FSensor_orientation != "horizontal" && FSensor_orientation != "vertical") {
        std::cout << "Error in sensor orientation parameter..." << std::endl;
        exit(EXIT_FAILURE);
    }
    if(this->get_parameter("publish_ground_point_cloud").as_bool()) {
        FPublisher_ground = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_ground", 10);  // Create publisher for ground points
    }
    count_errors_ransac = 0;
}

void DepthSubscriber::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    //! Get sensor orientation
    geometry_msgs::msg::TransformStamped tf;
    tf = FBuffer.lookupTransform("map", "depth_sensor", rclcpp::Time(0));
    Quaternion sensor_orientation = findGroundNormal(-tf.transform.rotation.x, -tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);

    //! Find the equation of the estimated ground plane
    Plane estimatedGround;
    estimatedGround.a = sensor_orientation.v.x;
    estimatedGround.b = sensor_orientation.v.y;
    estimatedGround.c = sensor_orientation.v.z;
    estimatedGround.d = FSensor_height;

    //! Read point cloud fields: x, y, z, rgba
    uint32_t total_bytes = msg->row_step * msg->height;
    for(int i = 0; i < total_bytes - 15;) {
        Point pt;
        union Float value_x{}, value_y{}, value_z{};
        //! Get x coordinate
        value_x.c[0] = msg->data[i];
        value_x.c[1] = msg->data[i+1];
        value_x.c[2] = msg->data[i+2];
        value_x.c[3] = msg->data[i+3];
        pt.x = value_x.f;
        //! Get y coordinate
        value_y.c[0] = msg->data[i+4];
        value_y.c[1] = msg->data[i+5];
        value_y.c[2] = msg->data[i+6];
        value_y.c[3] = msg->data[i+7];
        pt.y = value_y.f;
        //! Get z coordinate
        value_z.c[0] = msg->data[i+8];
        value_z.c[1] = msg->data[i+9];
        value_z.c[2] = msg->data[i+10];
        value_z.c[3] = msg->data[i+11];
        pt.z = value_z.f;
        //! Get r, g, b, a values
        pt.r = msg->data[i+14];
        pt.g = msg->data[i+13];
        pt.b = msg->data[i+12];
        pt.a = ((float)msg->data[i+15] / 255);
        //! Calculate the distance from each point to the plane and consider only the points that are very close to the estimated plane
        auto norm = std::sqrt(estimatedGround.a * estimatedGround.a + estimatedGround.b * estimatedGround.b + estimatedGround.c * estimatedGround.c);
        if (std::fabs(estimatedGround.a * pt.x + estimatedGround.b * pt.y + estimatedGround.c * pt.z + estimatedGround.d) / norm < 200) {
            estimatedGround.points.push_back(pt);  // Consider it as a point with a great possibility of belonging to the ground.
        }
        i += 16;
    }
    //! Find the best fitting ground plane
    Plane final_ground_plane;
    bool ret_ransac = RANSAC(estimatedGround, final_ground_plane);
    if(!ret_ransac) {
        count_errors_ransac++;
        return;
    }
    else if(count_errors_ransac > 0)
        count_errors_ransac = 0;
    //! Publish point cloud with ground points
    if(this->get_parameter("publish_ground_point_cloud").as_bool() && ret_ransac) {
        publishGroundPointCloud(final_ground_plane);
    }
    //! Evaluate danger
    uint8_t ret;
    if(count_errors_ransac == 5)
        ret = 2;  // Danger situation
    else
        ret = evaluateDanger(msg, total_bytes, final_ground_plane, sensor_orientation);

    if(ret == 1) {
        RCLCPP_INFO(get_logger(), "Too much inclination. DANGER!!!\n");
    }
    else if(ret == 2) {
        RCLCPP_INFO(get_logger(), "Invalid ground detection. Lack of points... DANGER!!!\n");
    }
    else if(ret == 3) {
        RCLCPP_INFO(get_logger(), "Probable obstacle. Be careful...\n");
    }
    else if(ret == 4) {
        RCLCPP_INFO(get_logger(), "Obstacle very close.. DANGER!!!\n");
    }
    else if(ret == 5) {
        RCLCPP_INFO(get_logger(), "Probable hole. Be careful...\n");
    }
    else if(ret == 6) {
        RCLCPP_INFO(get_logger(), "Hole very close. DANGER!!!\n");
    }
}

void DepthSubscriber::publishGroundPointCloud(Plane &ground_points)
{
    auto msg = sensor_msgs::msg::PointCloud2();  // Create message
    std::vector<sensor_msgs::msg::PointField> fields_ground;  // Store all message the data fields
    auto field_ground = sensor_msgs::msg::PointField();  // Specify message data field
    field_ground.name = "x";
    field_ground.datatype = 7;
    field_ground.count = 1;
    field_ground.offset = 0;
    fields_ground.push_back(field_ground);  // Create x data field

    field_ground.name = "y";
    field_ground.offset = 4;
    fields_ground.push_back(field_ground);  // Create y data field

    field_ground.name = "z";
    field_ground.offset = 8;
    fields_ground.push_back(field_ground);  // Create z data field

    field_ground.name = "rgb";
    field_ground.offset = 12;
    fields_ground.push_back(field_ground);  // Create rgba data field

    uint32_t num_points = ground_points.points.size();
    msg.header.frame_id = "map";  // Specify base frame
    msg.height = 1;
    msg.fields = fields_ground;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.is_dense = true;  // Contains only valid points
    msg.row_step = msg.point_step * num_points;
    msg.width = num_points;

    union Float xg{}, yg{}, zg{};
    for(int g = 0; g < num_points; g++) {
        xg.f = ground_points.points[g].x;
        yg.f = ground_points.points[g].y;
        zg.f = ground_points.points[g].z;
        for(unsigned char & k : xg.c) {
            msg.data.push_back(k);  // Append x data
        }
        for(unsigned char & k : yg.c) {
            msg.data.push_back(k);  // Append y data
        }
        for(unsigned char & k : zg.c) {
            msg.data.push_back(k);  // Append z data
        }
        //! Add color information for each point
        msg.data.push_back(ground_points.points[g].b);
        msg.data.push_back(ground_points.points[g].g);
        msg.data.push_back(ground_points.points[g].r);
        msg.data.push_back(ground_points.points[g].a);
    }
    FPublisher_ground->publish(msg);  // Publish message
}

//! Nullify rotation in z axis and return the quaternion with the normal vector to the ground
Quaternion DepthSubscriber::findGroundNormal(const float qx, const float qy, const float qz, const float qw)
{
    Quaternion q(qw, qx, qy, qz);
    Quaternion r(0,0,0,0);  // Initiate ground normal vector
    if(FSensor_orientation == "horizontal") {
        r.v.z = 1;  // Normal vector is (0,0,1)
    }
    else {
        r.v.x = -1; // The normal vector to ground when sensor is in the vertical is (-1,0,0)
        //! Put sensor in the horizontal
        Quaternion q_roll_minus90((float)std::sqrt(2)/2, 0, (float)std::sqrt(2)/2, 0);
        Quaternion new_q = q*q_roll_minus90;
        q.s = new_q.s;
        q.v.x = new_q.v.x;
        q.v.y = new_q.v.y;
        q.v.z = new_q.v.z;
    }
    Vector3D p1(1,0,0);  // Vector perpendicular to the axis we want to nullify (z axis)
    Quaternion p(0, p1);
    Quaternion qInverse(q.s, -q.v.x, -q.v.y, -q.v.z);
    Quaternion rotatedVector=q*p*qInverse;  // Rotate p1 by q to get p2
    Vector3D p2(rotatedVector.v.x, rotatedVector.v.y, rotatedVector.v.z);
    //! Find projection vector of p2 in xy plane and normalize the vector
    Vector3D projection(p2.x, p2.y, 0);  // Projection of p2 in xy_plane
    float magnitude = std::sqrt(projection.x * projection.x + projection.y * projection.y);
    projection.x = projection.x / magnitude;
    projection.y = projection.y / magnitude;
    //! Get angle from p1 to the projection vector
    float angle_difference = std::atan2(projection.y, projection.x);
    //! Create new quaternion to remove angle difference
    Quaternion remove_z_offset(std::cos(-angle_difference/2), 0, 0, std::sin(-angle_difference/2));
    //! Create new quaternion that combines original rotation with one that removes rotation around z
    Quaternion final_quaternion = remove_z_offset * q;
    if(FSensor_orientation == "vertical") {
        //! Change x and z values
        float aux = final_quaternion.v.x;
        final_quaternion.v.x = final_quaternion.v.z;
        final_quaternion.v.z = aux;
    }
    Quaternion qFinalInverse(final_quaternion.s, -final_quaternion.v.x, -final_quaternion.v.y, -final_quaternion.v.z);
    Quaternion groundNormal=final_quaternion*r*qFinalInverse;

    return groundNormal;
}

bool DepthSubscriber::RANSAC(const Plane& in_plane, Plane& final_ground_plane)
{
    int   max_idx       = in_plane.points.size();
    int   min_idx       = 0;
    int   max_tries     = 1000;
    int   c_max_inliers = 0;
    int max_iters       = 20;
    for (auto k = 0; k < max_iters; k++) {
        //! Declare private point cloud to store current solution
        std::vector<Point> m_pcl;
        //! Reset number of inliers in each iteration
        int num_inliers = 0;
        //! Randomly select three points that cannot be cohincident
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
            return false;
        }
        //! Declare the 3 points selected on this iteration
        Point pt1 = Point(
                in_plane.points[idx1].x, in_plane.points[idx1].y, in_plane.points[idx1].z);
        Point pt2 = Point(
                in_plane.points[idx2].x, in_plane.points[idx2].y, in_plane.points[idx2].z);
        Point pt3 = Point(
                in_plane.points[idx3].x, in_plane.points[idx3].y, in_plane.points[idx3].z);
        //! Extract the plane hessian coefficients
        Vector3D v1(pt2, pt1);
        Vector3D v2(pt3, pt1);
        Vector3D abc = v1.cross(v2);
        float    a   = abc.x;
        float    b   = abc.y;
        float    c   = abc.z;
        float    d   = -(a * pt1.x + b * pt1.y + c * pt1.z);

        for (const auto& m_pt : in_plane.points) {
            //! Compute the distance from each point to the plane
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

uint8_t DepthSubscriber::evaluateDanger(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, uint32_t total_bytes, Plane& ground_plane, Quaternion sensor_orientation)
{
    //! Check if it is a danger situation
    uint32_t num_danger_points = 0, num_points_under_ground = 0;

    for(int i = 0; i < total_bytes - 15;) {
        Point pt;
        union Float value_x{}, value_y{}, value_z{};
        //! Get x coordinate
        value_x.c[0] = msg->data[i];
        value_x.c[1] = msg->data[i + 1];
        value_x.c[2] = msg->data[i + 2];
        value_x.c[3] = msg->data[i + 3];
        pt.x = value_x.f;
        if( (FSensor_orientation == "horizontal" && (pt.x < -300 || pt.x > 300)) || (FSensor_orientation == "vertical" && pt.x < -500) ) {
            i += 16;
            continue;
        }
        //! Get y coordinate
        value_y.c[0] = msg->data[i + 4];
        value_y.c[1] = msg->data[i + 5];
        value_y.c[2] = msg->data[i + 6];
        value_y.c[3] = msg->data[i + 7];
        pt.y = value_y.f;
        //! Get z coordinate
        value_z.c[0] = msg->data[i + 8];
        value_z.c[1] = msg->data[i + 9];
        value_z.c[2] = msg->data[i + 10];
        value_z.c[3] = msg->data[i + 11];
        pt.z = value_z.f;
        if( (FSensor_orientation == "vertical" && (pt.z < -300 || pt.z > 300)) || (FSensor_orientation == "horizontal" && pt.z > 500) ) {
            i += 16;
            continue;
        }

        if( (FSensor_orientation == "horizontal" && ground_plane.c < 0) || (FSensor_orientation == "vertical" && ground_plane.a > 0) ) {
            ground_plane.a = -ground_plane.a;
            ground_plane.b = -ground_plane.b;
            ground_plane.c = -ground_plane.c;
            ground_plane.d = -ground_plane.d;
        }
        //! Calculate the distance from the point to the plane
        auto norm = std::sqrt(ground_plane.a * ground_plane.a + ground_plane.b * ground_plane.b + ground_plane.c * ground_plane.c);
        if (std::fabs(ground_plane.a * pt.x + ground_plane.b * pt.y + ground_plane.c * pt.z + ground_plane.d) / norm > 60) {
            //! Calculate the distance from the point to the sensor
            float distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if(distance < 740) {
                num_danger_points++;
            }
            //printf("%f %f %f %f\n", ground_plane.a/norm, ground_plane.b/norm, ground_plane.c/norm, ground_plane.d/norm);

            if((ground_plane.a * pt.x + ground_plane.b * pt.y + ground_plane.c * pt.z > 60) && pt.y < 1500) {  // Point is below the plane
                num_points_under_ground++;
            }
        }
        i += 16;
    }
    //printf("Danger Points: %u %u\n", num_danger_points, num_points_under_ground);  // TODO: Delete

    uint32_t minimum_points1 = (uint32_t)this->get_parameter("evaluate_danger.minimum_points_level1").as_int();
    uint32_t minimum_points2 = (uint32_t)this->get_parameter("evaluate_danger.minimum_points_level2").as_int();
    if(minimum_points2 < minimum_points1) {  // Check if there are no errors
        std::cout << "Error! Please check the parameters for danger evaluation that you specified..." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    // Evaluate danger
    //! Check if the sensor is too tilted
    float angle;
    if(FSensor_orientation == "horizontal") {
        angle = std::atan2(sensor_orientation.v.z, sensor_orientation.v.y) - M_PI/2;
    }
    else {
        angle = std::atan2(sensor_orientation.v.y, sensor_orientation.v.x);
        if(angle < 0) angle += M_PI;
        else angle -= M_PI;
    }
    if(FSensor_orientation == "vertical" && (std::abs(sensor_orientation.v.z) > std::sin(FSensor_max_inclination) || (angle < FSensor_normal_inclination - FSensor_max_inclination) || (angle > FSensor_normal_inclination + FSensor_max_inclination) ) )
        return 1;  // Danger
    if(FSensor_orientation == "horizontal" && (std::abs(sensor_orientation.v.x) > std::sin(FSensor_max_inclination) || (angle < FSensor_normal_inclination - FSensor_max_inclination) || (angle > FSensor_normal_inclination + FSensor_max_inclination) ))
        return 1;  // Danger
    //! Check if there is a valid ground detection
    if(ground_plane.points.size() <= 50000) {
        return 2; // Danger
    }
    //! Check if there are any obstacles above ground
    if(num_danger_points >= minimum_points1 && num_danger_points < minimum_points2)
        return 3;  // Possible danger
    else if(num_danger_points >= minimum_points2)
        return 4;  // Danger Situation
    //! Check if there is any hole near to the wheelchair
    if(num_points_under_ground > 25000 && num_points_under_ground < 40000) {
        return 5;  // Possible danger
    }
    else if(num_points_under_ground >= 40000) {
        return 6;  // Danger Situation
    }
    return 0;  // Safe situation
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto depth_node = std::make_shared<DepthSubscriber>();

    rclcpp::spin(depth_node);

    rclcpp::shutdown();
    return 0;
}
