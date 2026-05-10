# ROS2 TOGAKUSHI2 Workspace

TOGAKUSHI2のROS2ワークスペース

視線入力で操作できる車いす

WHILL Model CRとPCを接続し制御

ROS2humble, Ubuntu22.04LTS環境で動作確認

## セットアップ

```bash
# ワークスペースのルートに移動
cd ~/ros2_TOGAKUSHI2

# 必要パッケージのインストール
sudo apt install libhidapi-dev
sudo apt install libpcap-dev
sudo apt install libopencv-dev


# ビルド
colcon build
source install/setup.bash

# launchファイルで起動
ros2 launch togakushi2_pkg togakushi2_launch.py
```

## コマンド備忘録

```bash
## rqt_console
ros2 run rqt_console rqt_console
## ビルド結果全部消す
rm -rf build/ install/ log/
## テキストファイル作成
touch abstacle_node.py
## 選択ビルド
colcon build --packages-select py_pubsub
## pythonパッケージ作成
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
・setup.pyを編集
    entry_pointsを追加
・package.xmlを編集
##c++パッケージ作成
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub
```

### setup.py記入例

```python
from setuptools import find_packages, setup

package_name = 'game_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1e8ec3@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #ros2 run game_pkg game_node
            'game_node = game_pkg.game_node:main',
        ],
    },
)
```
