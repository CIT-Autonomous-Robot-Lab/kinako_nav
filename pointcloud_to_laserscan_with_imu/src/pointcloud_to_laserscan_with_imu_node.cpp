#include "pointcloud_to_laserscan_with_imu/pointcloud_to_laserscan_with_imu_node.h"

namespace pointcloud_to_laserscan_with_imu
{

  PointCloudToLaserScanWithIMUNode::PointCloudToLaserScanWithIMUNode()
      : Node("pointcloud_to_laserscan_with_imu_node"),
        scan_frame_id_("map"),
        min_z_(-0.2),
        max_z_(2.0),
        angle_increment_coefficient_(1)
  {
    this->declare_parameter<std::string>("scan_frame_id", scan_frame_id_);
    this->declare_parameter<double>("min_z", min_z_);
    this->declare_parameter<double>("max_z", max_z_);
    this->declare_parameter<int>("angle_increment_coefficient", angle_increment_coefficient_);

    this->get_parameter("scan_frame_id", scan_frame_id_);
    this->get_parameter("min_z", min_z_);
    this->get_parameter("max_z", max_z_);
    this->get_parameter("angle_increment_coefficient", angle_increment_coefficient_);

    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
    processed_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_pointcloud", 2);

    // IMUサブスクリプションの設定
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", 10,
        std::bind(&PointCloudToLaserScanWithIMUNode::imuCallback, this, std::placeholders::_1));

    // 点群サブスクリプションの設定
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&PointCloudToLaserScanWithIMUNode::cbPointCloud, this, std::placeholders::_1));
  }

  PointCloudToLaserScanWithIMUNode::~PointCloudToLaserScanWithIMUNode() {}

  void PointCloudToLaserScanWithIMUNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
  {
    latest_imu_msg_ = imu_msg;
  }

  void PointCloudToLaserScanWithIMUNode::cbPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (!latest_imu_msg_)
    {
      RCLCPP_WARN(get_logger(), "IMUデータが利用可能ではありません。");
      return;
    }

    auto laser_scan = convertPointCloudToLaserScanWithIMU(msg, latest_imu_msg_);
    if (laser_scan)
    {
      laser_scan_pub_->publish(*laser_scan);
    }
  }

  std::shared_ptr<sensor_msgs::msg::LaserScan> PointCloudToLaserScanWithIMUNode::convertPointCloudToLaserScan(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    std::vector<std::pair<float, float>> filtered_points;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      float z = *iter_z;
      if (z >= min_z_ && z <= max_z_)
      {
        float x = *iter_x;
        float y = *iter_y;
        filtered_points.emplace_back(x, y);
      }
    }

    auto processed_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    processed_scan->header = msg->header;
    processed_scan->header.frame_id = scan_frame_id_;
    processed_scan->angle_min = -M_PI;
    processed_scan->angle_max = M_PI;
    processed_scan->angle_increment = (processed_scan->angle_max - processed_scan->angle_min) / (filtered_points.size() > 0 ? filtered_points.size() : 1);
    processed_scan->range_min = 0.0;
    processed_scan->range_max = 100.0;

    size_t ranges_size = std::ceil(
        (processed_scan->angle_max - processed_scan->angle_min) /
        processed_scan->angle_increment);
    processed_scan->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    for (const auto &point : filtered_points)
    {
      double angle = std::atan2(point.second, point.first);
      double range = std::hypot(point.first, point.second);

      if (angle < processed_scan->angle_min || angle > processed_scan->angle_max)
      {
        continue;
      }

      int index = static_cast<int>(
          (angle - processed_scan->angle_min) / processed_scan->angle_increment);

      if (index >= 0 && index < static_cast<int>(ranges_size))
      {
        if (range < processed_scan->ranges[index])
        {
          processed_scan->ranges[index] = static_cast<float>(range);
        }
      }
    }

    return processed_scan;
  }

  std::shared_ptr<sensor_msgs::msg::LaserScan> PointCloudToLaserScanWithIMUNode::convertPointCloudToLaserScanWithIMU(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
      const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
  {
    tf2::Matrix3x3 rot_matrix = createRotationMatrix(imu_msg);
    std::vector<std::tuple<float, float, float>> filtered_points;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      float x = *iter_x;
      float y = *iter_y;
      float z_val = *iter_z;

      tf2::Vector3 point(x, y, z_val);
      tf2::Vector3 rotated_point = rot_matrix * point;

      if (rotated_point.z() >= min_z_ && rotated_point.z() <= max_z_)
      {
        filtered_points.emplace_back(rotated_point.x(), rotated_point.y(), rotated_point.z());
      }
    }

    publishProcessedPointCloud(msg->header, filtered_points);

    return buildLaserScan(msg->header, filtered_points);
  }

  double PointCloudToLaserScanWithIMUNode::getYawFromIMU(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) const
  {
    tf2::Quaternion q(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  tf2::Matrix3x3 PointCloudToLaserScanWithIMUNode::createRotationMatrix(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) const
  {
    tf2::Quaternion q(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w);
    double roll, pitch, _;
    tf2::Matrix3x3(q).getRPY(roll, pitch, _);

    tf2::Matrix3x3 rot_matrix;
    rot_matrix.setRPY(roll, pitch, 0.0);
    RCLCPP_INFO(get_logger(), "imu_msg->orientation.x: %f, imu_msg->orientation.y: %f, imu_msg->orientation.z: %f", imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    RCLCPP_INFO(get_logger(), "createRotationMatrix: %f, %f, %f", roll * 180 / M_PI, pitch * 180 / M_PI, _ * 180 / M_PI);
    return rot_matrix;
  }

  void PointCloudToLaserScanWithIMUNode::publishProcessedPointCloud(
      const std_msgs::msg::Header &header,
      const std::vector<std::tuple<float, float, float>> &filtered_points)
  {
    sensor_msgs::msg::PointCloud2 processed_pointcloud;
    processed_pointcloud.header = header;
    processed_pointcloud.header.frame_id = scan_frame_id_;
    processed_pointcloud.height = 1;
    processed_pointcloud.width = static_cast<uint32_t>(filtered_points.size());
    processed_pointcloud.is_dense = false;
    processed_pointcloud.is_bigendian = false;

    // フィールドの定義
    processed_pointcloud.fields.clear();
    addPointField(processed_pointcloud, "x", sensor_msgs::msg::PointField::FLOAT32, 0);
    addPointField(processed_pointcloud, "y", sensor_msgs::msg::PointField::FLOAT32, 4);
    addPointField(processed_pointcloud, "z", sensor_msgs::msg::PointField::FLOAT32, 8);
    addPointField(processed_pointcloud, "intensity", sensor_msgs::msg::PointField::FLOAT32, 12);

    processed_pointcloud.point_step = sizeof(float) * 4; // x, y, z, intensity
    processed_pointcloud.row_step = processed_pointcloud.point_step * processed_pointcloud.width;
    processed_pointcloud.data.reserve(processed_pointcloud.row_step);

    for (const auto &point : filtered_points)
    {
      float x = std::get<0>(point);
      float y = std::get<1>(point);
      float z = std::get<2>(point);
      float intensity = 0.0f; // 必要に応じて設定

      appendPointToData(processed_pointcloud.data, x);
      appendPointToData(processed_pointcloud.data, y);
      appendPointToData(processed_pointcloud.data, z);
      appendPointToData(processed_pointcloud.data, intensity);
    }

    processed_pointcloud_pub_->publish(processed_pointcloud);
  }

  void PointCloudToLaserScanWithIMUNode::addPointField(
      sensor_msgs::msg::PointCloud2 &pointcloud,
      const std::string &name,
      uint8_t datatype,
      uint32_t offset) const
  {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    pointcloud.fields.push_back(field);
  }

  template <typename T>
  void PointCloudToLaserScanWithIMUNode::appendPointToData(std::vector<uint8_t> &data, const T &value) const
  {
    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&value);
    data.insert(data.end(), ptr, ptr + sizeof(T));
  }

  std::shared_ptr<sensor_msgs::msg::LaserScan> PointCloudToLaserScanWithIMUNode::buildLaserScan(
      const std_msgs::msg::Header &header,
      const std::vector<std::tuple<float, float, float>> &filtered_points) const
  {
    auto processed_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    processed_scan->header = header;
    processed_scan->header.frame_id = scan_frame_id_;
    processed_scan->angle_min = -M_PI;
    processed_scan->angle_max = M_PI;
    processed_scan->angle_increment = M_PI / (angle_increment_coefficient_ * 180.0);
    processed_scan->range_min = 0.0;
    processed_scan->range_max = 100.0;

    size_t ranges_size = static_cast<size_t>(
        (processed_scan->angle_max - processed_scan->angle_min) /
        processed_scan->angle_increment);
    processed_scan->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    for (const auto &point : filtered_points)
    {
      double x = std::get<0>(point);
      double y = std::get<1>(point);
      double angle = std::atan2(y, x);
      double range = std::hypot(x, y);

      if (angle < processed_scan->angle_min || angle > processed_scan->angle_max)
      {
        continue;
      }

      int index = static_cast<int>(
          (angle - processed_scan->angle_min) / processed_scan->angle_increment);

      if (index >= 0 && index < static_cast<int>(ranges_size))
      {
        if (range < processed_scan->ranges[index])
        {
          processed_scan->ranges[index] = static_cast<float>(range);
        }
      }
    }

    return processed_scan;
  }

} // namespace pointcloud_to_laserscan_with_imu

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pointcloud_to_laserscan_with_imu::PointCloudToLaserScanWithIMUNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan_with_imu::PointCloudToLaserScanWithIMUNode)