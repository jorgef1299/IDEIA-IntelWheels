#include "zed2_camera.h"

ZED2_camera::ZED2_camera() : Node("zed2_camera")
{
    // Declare node parameters.
    this->declare_parameter<std::string>("mode", "ULTRA");
    this->declare_parameter<std::string>("unit", "MILLIMETER");
    this->declare_parameter<float>("minimum_distance", 400);
    this->declare_parameter<float>("max_range", 3000);
    this->declare_parameter<std::string>("topic_to_publish", "/depth_map");
    // Init all the parameters.
    init_parameters();
    std::string topic;
    this->get_parameter("topic_to_publish", topic);
    if(topic.empty()){
        std::cout << "ERROR in topic name! Exiting program..." << std::endl;
        exit(EXIT_FAILURE);
    }
    FPubDepth = image_transport::create_publisher(this, topic);
    // Open camera.
    auto returned_state = FZed.open(FInit_params);
    if(returned_state != sl::ERROR_CODE::SUCCESS){
        std::cout << "ERROR:" << returned_state << ", exiting program..." << std::endl;
        exit(EXIT_FAILURE);
    }
    // Enable position tracking
    sl::PositionalTrackingParameters tracking_parameters;
    FZed.enablePositionalTracking(tracking_parameters);

    FPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
}

ZED2_camera::~ZED2_camera()
{
    FZed.close();
}

void ZED2_camera::getDepthMap()
{
    if(FZed.grab() == sl::ERROR_CODE::SUCCESS) {
        FZed.retrieveMeasure(FDepth, sl::MEASURE::DEPTH);
        FZed.retrieveMeasure(FPointCloud, sl::MEASURE::XYZRGBA);
        // Get pose
        sl::POSITIONAL_TRACKING_STATE state = FZed.getPosition(FZed_pose, sl::REFERENCE_FRAME::WORLD);
    }
    else {
        std::cout << "ERROR: Cannot read ZED camera!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void ZED2_camera::init_parameters()
{
    std::string depth_mode, depth_unit;
    float min_distance, max_range;
    this->get_parameter("mode", depth_mode);
    this->get_parameter("unit", depth_unit);
    this->get_parameter("minimum_distance", min_distance);
    this->get_parameter("max_range", max_range);
    // Define depth mode.
    if(depth_mode == "PERFORMANCE"){
        FInit_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    }
    else if(depth_mode == "ULTRA"){
        FInit_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    }
    else if(depth_mode == "QUALITY"){
        FInit_params.depth_mode = sl::DEPTH_MODE::QUALITY;
    }
    else {
        std::cout << "Error in depth mode parameter..." << std::endl;
        exit(EXIT_FAILURE);
    }
    // Define depth unit.
    if(depth_unit == "MILLIMETER"){
        FInit_params.coordinate_units = sl::UNIT::MILLIMETER;
    }
    else if(depth_unit == "CENTIMETER"){
        FInit_params.coordinate_units = sl::UNIT::CENTIMETER;
    }
    else if(depth_unit == "FOOT"){
        FInit_params.coordinate_units = sl::UNIT::FOOT;
    }
    else if(depth_unit == "INCH"){
        FInit_params.coordinate_units = sl::UNIT::INCH;
    }
    else {
        std::cout << "Error in depth unit parameter..." << std::endl;
        exit(EXIT_FAILURE);
    }
    // Set min and max range.
    FInit_params.depth_minimum_distance = min_distance;
    FInit_params.depth_maximum_distance = max_range;
    // Set coordinate system to be right-handed and z-up
    FInit_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
}

std::shared_ptr<sensor_msgs::msg::Image> imageToROSMsg(sl::Mat &img, std::string frameId, rclcpp::Time t)
{
    std::shared_ptr<sensor_msgs::msg::Image> imgMessage = std::make_shared<sensor_msgs::msg::Image>();

    imgMessage->header.stamp = t;
    imgMessage->header.frame_id = frameId;
    imgMessage->height = img.getHeight();
    imgMessage->width = img.getWidth();

    int num = 1; // for endianness detection
    imgMessage->is_bigendian = *(char *) &num != 1;
    imgMessage->step = img.getStepBytes();
    size_t size = imgMessage->step * imgMessage->height;
    uint8_t* data_ptr=nullptr;
    sl::MAT_TYPE dataType = img.getDataType();
    imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    data_ptr = (uint8_t*)img.getPtr<sl::float1>();
    imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);

    return imgMessage;
}

void ZED2_camera::publishDepthImage()
{
    rclcpp::Time timeStamp = slTime2Ros(FZed.getTimestamp(sl::TIME_REFERENCE::CURRENT),get_clock()->get_clock_type());
    auto depth_img = imageToROSMsg(FDepth, "ZED2 depth image", timeStamp);
    FPubDepth.publish(depth_img);
}

void ZED2_camera::publishTF()
{
    static tf2_ros::TransformBroadcaster FPubTF(this);
    // Get ZED pose
    float tx = FZed_pose.getTranslation().tx;
    float ty = FZed_pose.getTranslation().ty;
    float tz = FZed_pose.getTranslation().tz;
    float qx = FZed_pose.getOrientation().ox;
    float qy = FZed_pose.getOrientation().oy;
    float qz = FZed_pose.getOrientation().oz;
    float qw = FZed_pose.getOrientation().ow;
    // Create TF Message
    geometry_msgs::msg::TransformStamped tf_msg;
    //TODO: Fill tf_msgs's header
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "depth_sensor";
    tf_msg.transform.translation.x = tx;
    tf_msg.transform.translation.y = ty;
    tf_msg.transform.translation.z = tz;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;

    FPubTF.sendTransform(tf_msg);
}

void ZED2_camera::publishPointCloud() {
    int count = 0;
    auto msg = sensor_msgs::msg::PointCloud2();
    auto height = FPointCloud.getHeight();
    auto width = FPointCloud.getWidth();

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

    msg.header.frame_id = "map";
    msg.height = 1;
    msg.fields = fields;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.is_dense = true;

    for(int i = 0; i < width; i++) {
        for(int j = 0; j < height; j++) {
            sl::float4 pt;
            FPointCloud.getValue(i,j, &pt);

            float distance = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            //printf("D: %f\n", distance);
            if(distance > 0 && distance < 3000) {
                count++;
                // Convert float to byte[4]
                union Float value_x, value_y, value_z, value_rgb;
                value_x.m_float = pt.x;
                value_y.m_float = pt.y;
                value_z.m_float = pt.z;
                value_rgb.m_float = pt.w;
                //printf("%f %f %f\n", value_x.m_float, value_y.m_float, value_z.m_float);
                for(int k = 0; k < 4; k++) {
                    msg.data.push_back(value_x.m_bytes[k]);
                }
                for(int k = 0; k < 4; k++) {
                    msg.data.push_back(value_y.m_bytes[k]);
                }
                for(int k = 0; k < 4; k++) {
                    msg.data.push_back(value_z.m_bytes[k]);
                }
                msg.data.push_back(value_rgb.m_bytes[2]);
                msg.data.push_back(value_rgb.m_bytes[1]);
                msg.data.push_back(value_rgb.m_bytes[0]);
                msg.data.push_back(value_rgb.m_bytes[3]);
                /*for(int k = 0; k < 4; k++) {
                    msg.data.push_back(value_rgb.m_bytes[k]);
                }*/
            }
        }
    }
    msg.row_step = msg.point_step * count;
    msg.width = count;

    if(count > 0)
        FPublisher->publish(msg);
}

rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type) {
    uint64_t ts_nsec = t.getNanoseconds();
    auto sec = static_cast<uint32_t>(ts_nsec / 1000000000);
    auto nsec = static_cast<uint32_t>(ts_nsec % 1000000000);
    return rclcpp::Time(sec, nsec, clock_type);
}