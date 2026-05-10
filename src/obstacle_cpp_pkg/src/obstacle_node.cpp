// 障害物検知のノード
// 単位は[m]
// Python版から完全移植 (ROS2 Humble / C++17)

#include <array>
#include <cmath>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <togakushi2_interfaces/msg/obstacle.hpp>

class ObstacleNode : public rclcpp::Node
{
public:
  ObstacleNode()
  : Node("obstacle_node"),
    // 近距離検知パラメータ
    DETECT_RANGE_FRONT(1.4),   // 前方向[m]
    DETECT_RANGE_BACK(0.8),    // 後方向[m]
    WIDTH_HALF(0.3),           // 前後範囲の半幅[m]
    HEIGHT_MIN(0.1),           // 地面のノイズ除去[m]
    HEIGHT_MAX(1.7),           // 上限高さ[m]
    THRESHOLD(10.0),           // 障害物判定閾値
    R_SIDE(0.8),               // 左右検知範囲、半径[m]
    // 遠距離検知パラメータ
    GRID_SIZE_FAR(2.0),        // 遠距離検知のグリッド長[m]
    GRID_MAX(3.0),             // グリッド座標の最大値[m]
    THRESHOLD_FAR(5)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "rslidar_points", 10,
      std::bind(&ObstacleNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<togakushi2_interfaces::msg::Obstacle>(
      "obstacle_info", 10);

    pub_far_ = this->create_publisher<togakushi2_interfaces::msg::Obstacle>(
      "obstacle_info_far", 10);  // 遠方障害物用
  }

private:
  // -----------------------------------------------------------------------
  // コールバック
  // -----------------------------------------------------------------------
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
  {
    togakushi2_interfaces::msg::Obstacle detection_result;
    togakushi2_interfaces::msg::Obstacle detection_result_far;

    // [*,前方,*,右,*,*,後ろ,*,*,左]
    detection_result.data.assign(10, false);
    // [左前, 前, 右前, 右, 右後, 後, 左後, 左]
    detection_result_far.data.assign(8, false);

    filter_points(lidar_msg, detection_result, detection_result_far);

    pub_->publish(detection_result);
    pub_far_->publish(detection_result_far);
  }

  // -----------------------------------------------------------------------
  // 点群フィルタリング・方向別カウント
  // -----------------------------------------------------------------------
  void filter_points(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    togakushi2_interfaces::msg::Obstacle & result,
    togakushi2_interfaces::msg::Obstacle & result_far)
  {
    // 近距離: 1:前, 3:右, 6:後ろ, 9:左
    // Python版の += 0.1 (前方のみ) も含め完全保持
    std::map<int, double> count = {{1, 0.0}, {3, 0.0}, {6, 0.0}, {9, 0.0}};

    // 遠方: 0:左前, 1:前, 2:右前, 3:右, 4:右後, 5:後, 6:左後, 7:左
    std::array<int, 8> count_far{};
    count_far.fill(0);

    // PointCloud2 イテレータ (skip_nans=True 相当はNaNチェックで実装)
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z_raw = *iter_z;

      // NaN スキップ (skip_nans=True 相当)
      if (std::isnan(x) || std::isnan(y) || std::isnan(z_raw)) {
        continue;
      }

      // LiDAR設置高さオフセット[m]
      const float z = z_raw + 1.55f;

      // 高さ成分が範囲外なら何もしない
      if (!(HEIGHT_MIN <= z && z <= HEIGHT_MAX)) {
        continue;
      }

      // ----------------------------------------------------------------
      // 近距離検知
      // ----------------------------------------------------------------
      if ((-WIDTH_HALF <= y && y <= WIDTH_HALF) &&
          (1.0f <= x && x <= static_cast<float>(DETECT_RANGE_FRONT)))
      {
        // 前方 (Python版に合わせて +0.1)
        count[1] += 0.1;
      }
      else if ((-WIDTH_HALF <= y && y <= WIDTH_HALF) &&
               (static_cast<float>(-DETECT_RANGE_BACK) <= x && x <= 0.0f))
      {
        // 後方
        count[6] += 1.0;
      }
      else if ((y < -static_cast<float>(WIDTH_HALF)) &&
               (x > 0.0f) &&
               (x * x + y * y <= static_cast<float>(R_SIDE * R_SIDE)))
      {
        // 右
        count[3] += 1.0;
      }
      else if ((y > static_cast<float>(WIDTH_HALF)) &&
               (x > 0.0f) &&
               (x * x + y * y <= static_cast<float>(R_SIDE * R_SIDE)))
      {
        // 左
        count[9] += 1.0;
      }

      // ----------------------------------------------------------------
      // 遠方検知
      // ----------------------------------------------------------------
      const float inner = static_cast<float>(GRID_MAX - GRID_SIZE_FAR);  // 1.0
      const float gmax  = static_cast<float>(GRID_MAX);                  // 3.0

      // 前3方向
      if (inner <= x && x <= gmax) {
        if (inner <= y && y <= gmax) {
          count_far[0]++;  // 左前
        } else if (-inner <= y && y <= inner) {
          count_far[1]++;  // 前
        } else if (-gmax <= y && y <= -inner) {
          count_far[2]++;  // 右前
        }
      }
      // 右・左
      else if (-inner <= x && x <= inner) {
        if (-gmax <= y && y <= -inner) {
          count_far[3]++;  // 右
        } else if (inner <= y && y <= gmax) {
          count_far[7]++;  // 左
        }
      }
      // 後3方向
      else if (-gmax <= x && x <= -inner) {
        if (-gmax <= y && y <= -inner) {
          count_far[4]++;  // 右後
        } else if (-inner <= y && y <= inner) {
          count_far[5]++;  // 後
        } else if (inner <= y && y <= gmax) {
          count_far[6]++;  // 左後
        }
      }
    }

    // 点数が閾値より多かったら障害物
    for (auto & [idx, cnt] : count) {
      result.data[static_cast<std::size_t>(idx)] = (cnt >= THRESHOLD);
    }
    for (int i = 0; i < 8; ++i) {
      result_far.data[static_cast<std::size_t>(i)] =
        (count_far[static_cast<std::size_t>(i)] >= THRESHOLD_FAR);
    }
  }

  // -----------------------------------------------------------------------
  // メンバ変数
  // -----------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<togakushi2_interfaces::msg::Obstacle>::SharedPtr pub_;
  rclcpp::Publisher<togakushi2_interfaces::msg::Obstacle>::SharedPtr pub_far_;

  // 近距離検知パラメータ
  const double DETECT_RANGE_FRONT;
  const double DETECT_RANGE_BACK;
  const double WIDTH_HALF;
  const double HEIGHT_MIN;
  const double HEIGHT_MAX;
  const double THRESHOLD;
  const double R_SIDE;

  // 遠距離検知パラメータ
  const double GRID_SIZE_FAR;
  const double GRID_MAX;
  const int    THRESHOLD_FAR;
};

// =========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleNode>());
  rclcpp::shutdown();
  return 0;
}