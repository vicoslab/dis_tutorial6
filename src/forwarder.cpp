#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dis.hpp>

using std::placeholders::_1;

// this is a simple node that only listens to camera point cloud and forwards it to another topic

rins_forwarder::rins_forwarder(const rclcpp::NodeOptions& options) : Node("rins_forwarder", options) {
    std::cout << "forwarder starting" << std::endl;

    param_topic_pointcloud_in = std::string("/oakd/rgb/preview/depth/points");
    param_topic_pointcloud_out = std::string("/rins_forwarder");
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10, std::bind(&rins_forwarder::topic_callback, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out, 2);
}

void rins_forwarder::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { publisher_->publish(*msg); }

int main(int argc, char* argv[]) {
    const rclcpp::NodeOptions options;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rins_forwarder>());
    rclcpp::shutdown();
    return 0;
}