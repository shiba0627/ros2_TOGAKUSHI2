#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <togakushi2_interfaces/msg/obstacle.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nlohmann/json.hpp>
#include <cmath>
#include <vector>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;

class ObstacleNode : public rclcpp::Node {
public:
  ObstacleNode() : rclcpp::Node("obstacle_node") {
    // Publisher
    pub_ = this->create_publisher<togakushi2_interfaces::msg::Obstacle>(
        "obstacle_info", 10);
    pub_points_ = this->create_publisher<std_msgs::msg::String>(
        "obstacle_points", 10);

    // Subscription
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "rslidar_points", 10,
        std::bind(&ObstacleNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ObstacleNode initialized");
  }

private:
  // Parameters
  const float DETECT_RANGE_FRONT = 1.4f;
  const float DETECT_RANGE_BACK = 0.8f;
  const float WIDTH_HALF = 0.3f;
  const float HEIGHT_MIN = 0.1f;
  const float HEIGHT_MAX = 1.7f;
  const float THRESHOLD = 10.0f;
  const float R_SIDE = 1.0f;
  const float LIDAR_HEIGHT = 1.55f;
  const float POINTS_MAX_RANGE = 1.5f;

  // Publishers
  rclcpp::Publisher<togakushi2_interfaces::msg::Obstacle>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_points_;

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  // Message
  togakushi2_interfaces::msg::Obstacle detection_result_;

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Filter points near obstacles
    filterPointsNear(cloud);
    pub_->publish(detection_result_);

    // Generate points
    std_msgs::msg::String points_msg;
    points_msg.data = generatePoints(cloud);
    pub_points_->publish(points_msg);
  }

  void filterPointsNear(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    // Initialize detection result
    detection_result_.data.assign(10, false);

    // Count points in each direction
    std::map<int, float> count;
    count[1] = 0.0f;  // front
    count[3] = 0.0f;  // right
    count[6] = 0.0f;  // back
    count[9] = 0.0f;  // left

    for (const auto& point : cloud.points) {
      float x = point.x;
      float y = point.y;
      float z = point.z + LIDAR_HEIGHT;

      // Height filter
      if (z < HEIGHT_MIN || z > HEIGHT_MAX) {
        continue;
      }

      // Front
      if (y >= -WIDTH_HALF && y <= WIDTH_HALF && x >= 1.0f &&
          x <= DETECT_RANGE_FRONT) {
        count[1] += 0.1f;
      }
      // Back
      else if (y >= -WIDTH_HALF && y <= WIDTH_HALF && x >= -DETECT_RANGE_BACK &&
               x <= 0.0f) {
        count[6] += 1.0f;
      }
      // Right
      else if (y < -WIDTH_HALF && x > 0.0f &&
               (x * x + y * y) <= (R_SIDE * R_SIDE)) {
        count[3] += 1.0f;
      }
      // Left
      else if (y > WIDTH_HALF && x > 0.0f &&
               (x * x + y * y) <= (R_SIDE * R_SIDE)) {
        count[9] += 1.0f;
      }
    }

    // Set detection result based on threshold
    for (const auto& [idx, cnt] : count) {
      detection_result_.data[idx] = (cnt >= THRESHOLD);
    }
  }

  std::string generatePoints(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    json json_data;
    json_data["points"] = json::array();

    for (const auto& point : cloud.points) {
      float x = point.x;
      float y = point.y;
      float z = point.z + LIDAR_HEIGHT;

      // Height filter
      if (z < HEIGHT_MIN || z > HEIGHT_MAX) {
        continue;
      }

      // Range filter (3x3m = ±1.5m)
      if (x < -POINTS_MAX_RANGE || x > POINTS_MAX_RANGE || y < -POINTS_MAX_RANGE ||
          y > POINTS_MAX_RANGE) {
        continue;
      }

      // Add point with 3 decimal places precision
      json point_json;
      point_json["x"] = std::round(x * 1000.0f) / 1000.0f;
      point_json["y"] = std::round(y * 1000.0f) / 1000.0f;
      json_data["points"].push_back(point_json);
    }

    return json_data.dump();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}