#ifndef POINTCLOUD_TO_LASERSCAN_WITH_IMU_NODE_H_
#define POINTCLOUD_TO_LASERSCAN_WITH_IMU_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <vector>
#include <tuple>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <limits>

namespace pointcloud_to_laserscan_with_imu
{

  class PointCloudToLaserScanWithIMUNode : public rclcpp::Node
  {
  public:
    PointCloudToLaserScanWithIMUNode();
    ~PointCloudToLaserScanWithIMUNode();

  private:
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
    void cbPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    std::shared_ptr<sensor_msgs::msg::LaserScan> convertPointCloudToLaserScan(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    std::shared_ptr<sensor_msgs::msg::LaserScan> convertPointCloudToLaserScanWithIMU(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

    double getYawFromIMU(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) const;
    tf2::Matrix3x3 createRotationMatrix(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) const;

    // PointCloud処理関数
    void publishProcessedPointCloud(
        const std_msgs::msg::Header &header,
        const std::vector<std::tuple<float, float, float>> &filtered_points);
    void addPointField(
        sensor_msgs::msg::PointCloud2 &pointcloud,
        const std::string &name,
        uint8_t datatype,
        uint32_t offset) const;
    template <typename T>
    void appendPointToData(std::vector<uint8_t> &data, const T &value) const;
    std::shared_ptr<sensor_msgs::msg::LaserScan> buildLaserScan(
        const std_msgs::msg::Header &header,
        const std::vector<std::tuple<float, float, float>> &filtered_points) const;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pointcloud_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::shared_ptr<const sensor_msgs::msg::Imu> latest_imu_msg_;

    std::string scan_frame_id_;
    double min_z_;
    double max_z_;
    int angle_increment_coefficient_;
  };

} // namespace pointcloud_to_laserscan_with_imu

#endif // POINTCLOUD_TO_LASERSCAN_WITH_IMU_NODE_H_
