#include "zed2_camera.h"

ZED2_camera::ZED2_camera() : Node("zed2_camera")
{
    //! Declare node parameters.
    this->declare_parameter<std::string>("mode", "ULTRA");
    this->declare_parameter<std::string>("unit", "MILLIMETER");
    this->declare_parameter<float>("minimum_distance", 400);
    this->declare_parameter<float>("max_range", 3000);
    this->declare_parameter<std::string>("topic_to_publish", "/point_cloud");
    //! Init all the parameters.
    init_parameters();
    std::string topic;
    this->get_parameter("topic_to_publish", topic);
    if(topic.empty()){
        std::cout << "ERROR in topic name! Exiting program..." << std::endl;
        exit(EXIT_FAILURE);
    }
    //! Open camera.
    auto returned_state = FZed.open(FInit_params);
    if(returned_state != sl::ERROR_CODE::SUCCESS){
        std::cout << "ERROR:" << returned_state << ", exiting program..." << std::endl;
        exit(EXIT_FAILURE);
    }
    //! Enable position tracking
    sl::PositionalTrackingParameters tracking_parameters;
    FZed.enablePositionalTracking(tracking_parameters);

    FPublisher = create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("topic_to_publish").as_string(), 10);
}

ZED2_camera::~ZED2_camera()
{
    FZed.close();
}

void ZED2_camera::getData()
{
    if(FZed.grab() == sl::ERROR_CODE::SUCCESS) {
        FZed.retrieveMeasure(FPointCloud, sl::MEASURE::XYZRGBA);  // Get Point Cloud
        //! Get pose
        sl::POSITIONAL_TRACKING_STATE state = FZed.getPosition(FZed_pose, sl::REFERENCE_FRAME::WORLD);
    }
    else {
        std::cout << "ERROR: Cannot read ZED camera!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void ZED2_camera::init_parameters()
{
    std::string depth_mode, distance_unit;
    float min_distance;
    //! Read parameter values
    this->get_parameter("mode", depth_mode);
    this->get_parameter("unit", distance_unit);
    this->get_parameter("minimum_distance", min_distance);
    this->get_parameter("max_range", FMaxRange);
    //! Define depth mode.
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
    //! Define distance unit.
    if(distance_unit == "MILLIMETER"){
        FInit_params.coordinate_units = sl::UNIT::MILLIMETER;
    }
    else if(distance_unit == "CENTIMETER"){
        FInit_params.coordinate_units = sl::UNIT::CENTIMETER;
    }
    else if(distance_unit == "FOOT"){
        FInit_params.coordinate_units = sl::UNIT::FOOT;
    }
    else if(distance_unit == "INCH"){
        FInit_params.coordinate_units = sl::UNIT::INCH;
    }
    else {
        std::cout << "Error in distance unit parameter..." << std::endl;
        exit(EXIT_FAILURE);
    }
    //! Set min and max range.
    if(min_distance <= 0 || min_distance > 5000 || FMaxRange <= min_distance) {
        std::cout << "Error in minimum or maximum distance value parameters..." << std::endl;
        exit(EXIT_FAILURE);
    }
    FInit_params.depth_minimum_distance = min_distance;
    FInit_params.depth_maximum_distance = FMaxRange;

    //! Set coordinate system
    FInit_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
}

void ZED2_camera::publishTF()
{
    static tf2_ros::TransformBroadcaster FPubTF(this);
    //! Get ZED pose
    float tx = FZed_pose.getTranslation().tx;
    float ty = FZed_pose.getTranslation().ty;
    float tz = FZed_pose.getTranslation().tz;
    float qx = FZed_pose.getOrientation().ox;
    float qy = FZed_pose.getOrientation().oy;
    float qz = FZed_pose.getOrientation().oz;
    float qw = FZed_pose.getOrientation().ow;
    //! Create tf Message
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "depth_sensor";
    tf_msg.transform.translation.x = tx;
    tf_msg.transform.translation.y = ty;
    tf_msg.transform.translation.z = tz;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    //! Publish tf message
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
            if(distance > 0 && distance < FMaxRange) {
                count++;
                //! Convert float to byte[4]
                union Float value_x, value_y, value_z, value_rgb;
                value_x.m_float = pt.x;
                value_y.m_float = pt.y;
                value_z.m_float = pt.z;
                value_rgb.m_float = pt.w;

                for(int k = 0; k < 4; k++) {
                    msg.data.push_back(value_x.m_bytes[k]);  // pt.x bytes
                }
                for(int k = 0; k < 4; k++) {
                    msg.data.push_back(value_y.m_bytes[k]);  // pt.y bytes
                }
                for(int k = 0; k < 4; k++) {
                    msg.data.push_back(value_z.m_bytes[k]);  // pt.z bytes
                }
                msg.data.push_back(value_rgb.m_bytes[2]);  // r
                msg.data.push_back(value_rgb.m_bytes[1]);  // g
                msg.data.push_back(value_rgb.m_bytes[0]);  // b
                msg.data.push_back(value_rgb.m_bytes[3]);  // a
            }
        }
    }
    msg.row_step = msg.point_step * count;
    msg.width = count;
    //! Check if there are points to publish
    if(count > 0) {
        FPublisher->publish(msg);  // Publish Point Cloud
    }
}