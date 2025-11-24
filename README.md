# ROS2 TOGAKUSHI2 Workspace
TOGAKUSHI2のROS2ワークスペース

ROS2humble, Ubuntu22.04LTS環境で動作確認

## セットアップ
```bash
# ワークスペースのルートに移動
cd ~/ros2_TOGAKUSHI2

# 必要パッケージのインストール
sudo apt install libhidapi-dev
sudo apt install libpcap-dev

# ビルド
colcon build

# launchファイルで起動
ros2 launch togakushi2_pkg togakushi2_launch.py
```