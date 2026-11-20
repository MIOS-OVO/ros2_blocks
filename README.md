# ROS 2 彩色方块识别（C++）

工作区在 Ubuntu 虚拟机的 `~/rosfresh/ros2_blocks`。用 VS Code 打开该文件夹，在集成终端执行以下命令。工程自带用户提供的 1706 × 1279 样图。这台虚拟机可以用以下命令再次打开项目：

```bash
~/.local/opt/VSCode-linux-arm64/bin/code ~/rosfresh/ros2_blocks
```

## 编译与启动

虚拟机已装好 ROS 2 Humble、OpenCV 开发库和 Foxglove Bridge。换到另一台已安装 Humble 的 Ubuntu 22.04 时，如果缺少依赖，先补装：

```bash
sudo apt install build-essential libopencv-dev python3-colcon-common-extensions \
  ros-humble-ros-base ros-humble-foxglove-bridge
```

在第一个终端运行，启动后保持终端运行：

```bash
cd ~/rosfresh/ros2_blocks
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 run recognized_blocks bringup
```

在第二个终端运行：

```bash
cd ~/rosfresh/ros2_blocks
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run recognized_blocks select_color
```

输入 `红`、`蓝`、`绿` 或 `red`、`blue`、`green` 后回车，每次识别并框出一个颜色。终端会返回框的左上角坐标和宽高，单位为像素。输入 `q` 退出选色。也可在 VS Code 命令面板选择 **Tasks: Run Task**，依次运行 **ROS 2: Build**、**ROS 2: Launch detection and Foxglove**、**ROS 2: Select block color**。如果 VS Code 处于 Restricted Mode，也可以直接用两个 Ubuntu 终端运行上面的命令。

可用下面的命令单独验证服务：

```bash
ros2 service call /detect_block block_interfaces/srv/DetectBlock "{color: green}"
```

## 在 Foxglove 查看画面

1. 在 Ubuntu 虚拟机里打开 Foxglove 桌面应用，或在支持的浏览器里打开 [Foxglove 网页版](https://app.foxglove.dev/)。
2. 选 **Open connection → Foxglove WebSocket**，输入 `ws://localhost:8765`。
3. 添加 **Image（图像）**面板，在面板设置中将 Topic 设为 `/camera/compressed`。
4. 在面板设置的 **Image annotations（图像注释）**中启用 `/camera/block_marker`。可打开 **Sync timestamps（同步时间戳）**，图片和矩形使用相同的 ROS 时间戳。
5. 回到选色终端，依次输入 `红`、`蓝`、`绿`。红框应依次在左侧、中间、右侧方块上出现。

若 Foxglove 在 Mac 上运行，默认 Bridge 只监听虚拟机内部。可以在 Mac 终端使用下面的 SSH 转发命令并保持运行，然后在 Mac 的 Foxglove 中连接相同的 `ws://localhost:8765`。虚拟机 IP 变化时，以 Ubuntu 中 `ip -br addr` 的输出为准。

```bash
ssh -N -L 8765:127.0.0.1:8765 mios@192.168.190.129
```

## 通信和源码

| 接口 | 类型 | 数据流 |
| --- | --- | --- |
| `/detect_block` | `block_interfaces/srv/DetectBlock` | C++ 终端客户端提交颜色，识别节点返回 `found`、`message` 和像素坐标 `x/y/width/height` |
| `/camera/compressed` | `sensor_msgs/msg/CompressedImage` | 识别节点每 0.5 秒发布一帧样图 JPEG |
| `/camera/block_marker` | `visualization_msgs/msg/ImageMarker` | 识别节点发布同帧的红色矩形，Foxglove 叠加在样图上 |

Foxglove Bridge 负责把 ROS 2 话题转发到 WebSocket。选色服务在 ROS 2 内直接通信，发送请求后识别节点会立即发布更新后的矩形。识别代码在 `src/recognized_blocks/src/detector.cpp`，终端客户端在同目录的 `select_color.cpp`。识别沿用 HSV 分割、形态学处理和轮廓筛选。启动程序在同目录的 `bringup.cpp`，负责启动和关闭识别节点与 Bridge。在启动终端按 `Ctrl+C` 可同时停止这两个进程。换一张静态图时，给启动命令追加 `--image-path "/绝对路径/图片.png"`。需要改端口时，追加 `--port 8766`，并在 Foxglove 中连接 `ws://localhost:8766`。

## 样图实测结果

| 颜色 | 位置 | `x` | `y` | `width` | `height` |
| --- | --- | ---: | ---: | ---: | ---: |
| 红 | 左 | 288 | 114 | 307 | 311 |
| 蓝 | 中 | 776 | 128 | 264 | 312 |
| 绿 | 右 | 1275 | 128 | 337 | 317 |

已在 Ubuntu 22.04 / ROS 2 Humble 下编译，并通过 Foxglove Bridge 的 WebSocket 验证每种颜色对应的 JPEG 和矩形坐标及时间戳。

[Foxglove 的 ROS 2 接入文档](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2) · [图像面板与图像注释](https://docs.foxglove.dev/docs/visualization/panels/image)
