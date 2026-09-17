#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>

#include "lidar/point_types.h"
#include "FreeDOM-ROS2/utils.h"
#include "FreeDOM-ROS2/common_types.h"

using namespace freedom;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("static_map_evaluate");
    Param param(*node);

    double voxel_size;
    std::string ground_truth_path;
    std::string static_map_path;

    param.getParam<double>("voxel_size", voxel_size, 0.2);
    param.getParam<std::string>("ground_truth_path", ground_truth_path);
    param.getParam<std::string>("static_map_path", static_map_path);

    double voxel_size_inv = 1.0 / voxel_size;

    unsigned int total_static_voxels = 0;
    unsigned int total_dynamic_voxels = 0;
    unsigned int preserved_static_voxels = 0;
    unsigned int preserved_dynamic_voxels = 0;

    RCLCPP_INFO(node->get_logger(), "Loading ground truth pcd...");
    pcl::PointCloud<evaluate_pcl::Point>::Ptr ground_truth_cloud(new pcl::PointCloud<evaluate_pcl::Point>);
    if (pcl::io::loadPCDFile<evaluate_pcl::Point>(ground_truth_path, *ground_truth_cloud) == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Couldn't read file %s", ground_truth_path.c_str());
        return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Ground truth pcd loaded, %li points", ground_truth_cloud->size());

    RCLCPP_INFO(node->get_logger(), "Loading static map pcd...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(static_map_path, *static_map_cloud) == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Couldn't read file %s", static_map_path.c_str());
        return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Static map pcd loaded, %li points", static_map_cloud->size());

    ProgressBar bar("Ground truth mapping", 40, ground_truth_cloud->size(), 100);
    std::unordered_map<Eigen::Vector3i, Label, IndexHash> voxel_map;
    for(const auto& point : *ground_truth_cloud)
    {
        bar.step();

        Eigen::Vector3d point_coord(point.x, point.y, point.z);
        Eigen::Vector3i voxel_idx(  std::floor(point_coord.x()*voxel_size_inv),
                                    std::floor(point_coord.y()*voxel_size_inv),
                                    std::floor(point_coord.z()*voxel_size_inv));
        Label label = static_cast<Label>(point.label);
        voxel_map.insert(std::make_pair(voxel_idx, label));
        if(label == LABEL_STATIC)
            total_static_voxels ++;
        else if(label == LABEL_DYNAMIC)
            total_dynamic_voxels ++;
        else
            RCLCPP_ERROR(node->get_logger(), "label value error");
    }

    ProgressBar bar1("Static map mapping", 40, static_map_cloud->size(), 100);
    std::unordered_set<Eigen::Vector3i, IndexHash> static_voxels;
    for(const auto& point : *static_map_cloud)
    {
        bar1.step();

        Eigen::Vector3d point_coord(point.x, point.y, point.z);
        Eigen::Vector3i voxel_idx(  std::floor(point_coord.x()*voxel_size_inv),
                                    std::floor(point_coord.y()*voxel_size_inv),
                                    std::floor(point_coord.z()*voxel_size_inv));

        static_voxels.insert(voxel_idx);
    }

    ProgressBar bar2("evaluating", 40, voxel_map.size(), 100);
    for(const auto& pair : voxel_map)
    {
        bar2.step();

        auto it = static_voxels.find(pair.first);
        if(it != static_voxels.end())
        {
            Label ground_truth_label = pair.second;
            if(ground_truth_label == LABEL_STATIC)
                preserved_static_voxels ++;
            else if(ground_truth_label == LABEL_DYNAMIC)
                preserved_dynamic_voxels ++;
        }
    }

    double PR = static_cast<double>(preserved_static_voxels) / total_static_voxels;
    double RR = double(1.0) - static_cast<double>(preserved_dynamic_voxels) / total_dynamic_voxels;
    double F1 = 2*((PR * RR)/(PR + RR));

    std::cout << "\ntotal_static_voxels: " << total_static_voxels << "\n";
    std::cout << "total_dynamic_voxels: " << total_dynamic_voxels << "\n";
    std::cout << "preserved_static_voxels: " << preserved_static_voxels << "\n";
    std::cout << "preserved_dynamic_voxels: " << preserved_dynamic_voxels;

    std::ostringstream PR_stream, RR_stream, F1_stream;
    PR_stream << std::fixed << std::setprecision(3) << PR * 100 << "%";
    RR_stream << std::fixed << std::setprecision(3) << RR * 100 << "%";
    F1_stream << std::fixed << std::setprecision(3) << F1 * 100 << "%";

    std::cout << "\n";
    std::cout << std::setfill('-');
    std::cout << std::setw(30) << "" << std::endl;

    std::cout << std::setfill(' ');
    std::cout << std::left << std::setw(10) << "PR"
              << std::setw(10) << "RR"
              << std::setw(10) << "F1" << std::endl;

    std::cout << std::setfill('-');
    std::cout << std::setw(30) << "" << std::endl;

    std::cout << std::setfill(' ');
    std::cout << std::left << std::setw(10) << PR_stream.str()
              << std::setw(10) << RR_stream.str()
              << std::setw(10) << F1_stream.str() << std::endl;

    std::cout << std::setfill('-');
    std::cout << std::setw(30) << "" << std::endl;

    rclcpp::shutdown();
    return 0;
}
