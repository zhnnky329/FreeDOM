#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "lidar/point_types.h"
#include "FreeDOM-ROS2/utils.h"
#include "FreeDOM-ROS2/common_types.h"

namespace freedom{

class GroundTruthGenerate : public rclcpp::Node{
public:
    GroundTruthGenerate(): Node("ground_truth_generate"), frame_count_(0)
    {
        Param param(*this);
        param.getParam<double>("voxel_size", voxel_size_, 0.4);
        param.getParam<double>("min_range", min_range_, 2.7);
        param.getParam<double>("max_range", max_range_, 1000.0);
        param.getParam<std::string>("map_tf_frame", map_tf_frame_);
        param.getParam<std::string>("sensor_tf_frame", sensor_tf_frame_);
        param.getParam<std::string>("pointcloud_topic", pointcloud_topic_);
        param.getParam<std::string>("save_map_topic", save_map_topic_);
        param.getParam<std::string>("save_map_path", save_map_path_);
        param.getParam<bool>("enable_save_frame", enable_save_frame_, false);
        param.getParam<std::string>("save_frame_path", save_frame_path_, std::string(""));

        min_range_squared_ = min_range_ * min_range_;
        max_range_squared_ = max_range_ * max_range_;
        voxel_size_half_ = voxel_size_ / 2.0;
        voxel_size_inv_ = 1.0 / voxel_size_;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::QoS(100),
            std::bind(&GroundTruthGenerate::pointcloud_callback, this, std::placeholders::_1));

        save_map_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            save_map_topic_, 10,
            std::bind(&GroundTruthGenerate::save_static_map_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud)
    {
        pcl::PointCloud<evaluate_pcl::Point>::Ptr cloud_ptr(new pcl::PointCloud<evaluate_pcl::Point>());
        pcl::fromROSMsg(*pointcloud, *cloud_ptr);
        if(cloud_ptr->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "pointcloud empty");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "lidar_transform:Pointcloud recieved,%zu points", cloud_ptr->points.size());

        try{
            rclcpp::Time transform_time = pointcloud->header.stamp;

            if(!tf_buffer_->canTransform(map_tf_frame_, sensor_tf_frame_, transform_time, rclcpp::Duration::from_seconds(10.0)))
            {
                RCLCPP_WARN(this->get_logger(), "no tf");
                return;
            }

            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_->lookupTransform(map_tf_frame_, sensor_tf_frame_, transform_time);

            Eigen::Isometry3d transform;
            transformfromTFToEigen(transformStamped, transform);
            Eigen::Vector3d point_pos_transformed;

            pcl::PointCloud<pcl::PointXYZ> frame_pc;

            for(auto& point : *cloud_ptr)
            {
                assert(point.label == 9 || point.label == 251);
                Label label = static_cast<Label>(point.label);

                Eigen::Vector3d point_pos(point.x, point.y, point.z);

                point_pos_transformed = transform * point_pos;

                double range_squared = (point_pos_transformed - transform.translation()).squaredNorm();

                if( range_squared < min_range_squared_ ||
                    range_squared > max_range_squared_)
                    continue;

                if(enable_save_frame_)
                {
                    pcl::PointXYZ frame_point(point_pos_transformed.x(), point_pos_transformed.y(), point_pos_transformed.z());
                    frame_pc.push_back(frame_point);
                }

                Eigen::Vector3i voxel_idx( std::floor(point_pos_transformed.x()*voxel_size_inv_),
                                    std::floor(point_pos_transformed.y()*voxel_size_inv_),
                                    std::floor(point_pos_transformed.z()*voxel_size_inv_));

                auto it = static_voxels_.find(voxel_idx);
                if(it != static_voxels_.end())
                {
                    if(it->second.first == LABEL_DYNAMIC && label == LABEL_STATIC)
                    {
                        static_voxels_[voxel_idx].first = label;
                        static_voxels_[voxel_idx].second = Eigen::Vector3d(point_pos_transformed.x(), point_pos_transformed.y(), point_pos_transformed.z());
                    }
                }
                else
                {
                    static_voxels_[voxel_idx].first = label;
                    static_voxels_[voxel_idx].second = Eigen::Vector3d(point_pos_transformed.x(), point_pos_transformed.y(), point_pos_transformed.z());
                }
            }

            if(enable_save_frame_)
            {
                frame_pc.sensor_origin_ = Eigen::Vector4f(transform.translation().x(), transform.translation().y(), transform.translation().z(), 1.0);
                pcl::io::savePCDFileASCII(save_frame_path_ + std::to_string(frame_count_) + ".pcd", frame_pc);
            }
        }
        catch(tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        ++frame_count_;
    }

    void save_static_map_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
    {
        pcl::PointCloud<evaluate_pcl::Point> pointcloud_point;
        pcl::PointCloud<evaluate_pcl::Point> pointcloud_voxel;
        pointcloud_point.reserve(static_voxels_.size());
        pointcloud_voxel.reserve(static_voxels_.size());

        for(const auto& it : static_voxels_)
        {
            evaluate_pcl::Point point;
            evaluate_pcl::Point voxel_point;

            point.x = it.second.second.x();
            point.y = it.second.second.y();
            point.z = it.second.second.z();
            point.label = static_cast<std::uint16_t>(it.second.first);

            voxel_point.x = it.first.x() * voxel_size_ + voxel_size_half_;
            voxel_point.y = it.first.y() * voxel_size_ + voxel_size_half_;
            voxel_point.z = it.first.z() * voxel_size_ + voxel_size_half_;
            voxel_point.label = static_cast<std::uint16_t>(it.second.first);

            pointcloud_point.push_back(point);
            pointcloud_voxel.push_back(voxel_point);
        }

        pointcloud_point.width = pointcloud_point.points.size();
        pointcloud_point.height = 1;
        pcl::io::savePCDFileASCII(save_map_path_ + "ground_truth_point.pcd", pointcloud_point);

        pointcloud_voxel.width = pointcloud_voxel.points.size();
        pointcloud_voxel.height = 1;
        pcl::io::savePCDFileASCII(save_map_path_ + "ground_truth_voxel.pcd", pointcloud_voxel);

        RCLCPP_INFO(this->get_logger(), "Static map saved at:%s", save_map_path_.c_str());
    }

    double voxel_size_;
    double voxel_size_half_;
    double voxel_size_inv_;
    double min_range_;
    double max_range_;
    double min_range_squared_;
    double max_range_squared_;
    std::string map_tf_frame_;
    std::string sensor_tf_frame_;
    std::string pointcloud_topic_;
    std::string save_map_topic_;
    std::string save_map_path_;
    bool enable_save_frame_;
    std::string save_frame_path_;

    uint64_t frame_count_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_map_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unordered_map<Eigen::Vector3i, std::pair<Label, Eigen::Vector3d>, IndexHash> static_voxels_;
};

} // namespace freedom

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<freedom::GroundTruthGenerate>());
    rclcpp::shutdown();
    return 0;
}
