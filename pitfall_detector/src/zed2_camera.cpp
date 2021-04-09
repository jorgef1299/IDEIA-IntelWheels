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
}

ZED2_camera::~ZED2_camera()
{
    FZed.close();
}

void ZED2_camera::getDepthMap()
{
    if(FZed.grab() == sl::ERROR_CODE::SUCCESS) {
        FZed.retrieveMeasure(FDepth, sl::MEASURE::DEPTH);
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

rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type) {
    uint64_t ts_nsec = t.getNanoseconds();
    auto sec = static_cast<uint32_t>(ts_nsec / 1000000000);
    auto nsec = static_cast<uint32_t>(ts_nsec % 1000000000);
    return rclcpp::Time(sec, nsec, clock_type);
}