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

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
using std::placeholders::_1;

void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_blob) {
    std::cout << "callback" << std::endl;

    // define objects
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(*cloud_blob, *pcl_pc2);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(pcl_pc2);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    pcl::IndicesPtr remaining(new std::vector<int>);
    remaining->resize(nr_points);
    for (size_t i = 0; i < remaining->size(); ++i) {
        (*remaining)[i] = static_cast<int>(i);
    }

    // While 30% of the original cloud is still there
    while (remaining->size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setIndices(remaining);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) break;

        // Extract the inliers
        std::vector<int>::iterator it = remaining->begin();
        for (size_t i = 0; i < inliers->indices.size(); ++i) {
            int curr = inliers->indices[i];
            // Remove it from further consideration.
            while (it != remaining->end() && *it < curr) {
                ++it;
            }
            if (it == remaining->end()) break;
            if (*it == curr) it = remaining->erase(it);
        }
        i++;
    }
    std::cout << "Found " << i << " planes." << std::endl;

    // Color all the non-planar things.
    for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it) {
        uint8_t r = 0, g = 255, b = 0;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);
    }

    sensor_msgs::msg::PointCloud2 cloud_out;
    pcl::PCLPointCloud2::Ptr cloud_final(new pcl::PCLPointCloud2()); // create PointCloud2
    pcl::toPCLPointCloud2(*cloud_filtered, *cloud_final); // 
    pcl_conversions::fromPCL(*cloud_final, cloud_out);
    pub->publish(cloud_out);

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::cout << "planes" << std::endl;

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("planes");

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    node->declare_parameter<std::string>("topic_pointcloud_in", "/oakd/rgb/preview/depth/points");
    std::string param_topic_pointcloud_in = node->get_parameter("topic_pointcloud_in").as_string();

    subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10, &cloud_cb);

    pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/planes", 2);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
