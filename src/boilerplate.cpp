#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { std::cout << "callback" << std::endl; }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::cout << "node" << std::endl;

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("name");

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    node->declare_parameter<std::string>("topic_pointcloud_in", "/oakd/rgb/preview/depth/points");
    std::string param_topic_pointcloud_in = node->get_parameter("topic_pointcloud_in").as_string();

    subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10, &cloud_cb);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
