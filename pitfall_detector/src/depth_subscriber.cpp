//TODO: Adicionar parâmetro para o tópico a subscrever
#include "depth_subscriber.h"

uint8_t last_state_show = 0; // used to close opencv windows only one time.

DepthSubscriber::DepthSubscriber() : Node("depth_subscriber")
{
    rclcpp::QoS depth_qos(10);
    depth_qos.keep_last(10);
    depth_qos.best_effort();
    depth_qos.durability_volatile();

    FSubscriber = create_subscription<sensor_msgs::msg::Image>("/depth_map", depth_qos, std::bind(&DepthSubscriber::depthCallback, this, _1));
    Node::declare_parameter("show_image", false);
}

DepthSubscriber::~DepthSubscriber()
{
    cv::destroyAllWindows();
}

void DepthSubscriber::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received image: %s", msg->header.frame_id.c_str());
    cv::Mat depth_image(msg->height, msg->width, CV_32FC1, const_cast<unsigned char *>(msg->data.data()), msg->step);
    if(Node::get_parameter("show_image").as_bool()) {
        this->showDepthMap(depth_image, msg->width, msg->height);
        last_state_show = 1;
    }

    else if(last_state_show == 1) {
        cv::destroyAllWindows();
        last_state_show = 0;
    }
}

void DepthSubscriber::showDepthMap(cv::Mat map, uint32_t width , uint32_t height)
{
    float max = 4000;
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            map.at<float>(i,j) = 1 - (map.at<float>(i,j) / max);
        }
    }
    cv::imshow("Depth map", map);
    cv::waitKey(1);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto depth_node = std::make_shared<DepthSubscriber>();

    rclcpp::spin(depth_node);

    rclcpp::shutdown();
    return 0;
}
