#include "zed2_camera.h"
#include <csignal>

sig_atomic_t stop_flag = 0;

void signalHandler(int signum)
{
    stop_flag = 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    auto zed_node = std::make_shared<ZED2_camera>();

    while(stop_flag == 0){
        zed_node->getDepthMap();
        //zed_node->publishDepthImage();
        zed_node->publishPointCloud();
        zed_node->publishTF();
    }
    //rclcpp::spin(zed_node);
    return 0;
}