camera_publisher

环境配置

- ROS2 Humble/Foxy

- C++编译器

- V4L2 (Video4Linux2) 接口

1. 打开终端，导航到工作空间目录：

   cd ~/ros2_ws

3. 清理之前的编译结果：

    rm -rf build install log

5. 编译所有包：

   colcon build

运行步骤
1. 打开终端，导航到工作空间目录：

   cd ~/ros2_ws

3. 设置环境变量：

    source install/setup.bash

4. 运行摄像头节点：

    ros2 run camera_publisher camera_publisher

注意事项

- 确保摄像头设备已连接并可用。

- 确保共享内存和信号量正确初始化。

- 如果遇到权限问题，请确保有权限访问摄像头设备（例如 `/dev/video0`）。# web_display

web_display

环境配置

- ROS2 Humble/Foxy

- C++编译器

- httplib 库

- libjpeg 库

编译步骤
1. 打开终端，导航到工作空间目录：
    ```bash
    cd ~/ros2_ws
    ```

2. 清理之前的编译结果：

    rm -rf build install log

3. 编译所有包：

    colcon build

运行步骤
1. 打开终端，导航到工作空间目录：

    cd ~/ros2_ws

2. 设置环境变量：

    source install/setup.bash

3. 运行Web显示节点：

    ros2 run web_display web_display

4. 在浏览器中访问 `http://localhost:8000/video` 查看实时视频流。

注意事项

- 确保摄像头节点已经在运行并发布图像数据。

- 确保共享内存和信号量正确初始化。

- 如果遇到权限问题，请确保有权限访问共享内存。

my_camera_msgs

环境配置

- ROS2 Humble/Foxy

- C++编译器

编译步骤

1. 打开终端，导航到工作空间目录：
 
    cd ~/ros2_ws

2. 清理之前的编译结果：

    rm -rf build install log

3. 编译所有包：
   
    colcon build

运行步骤

- 此包为消息定义包，无需单独运行。

注意事项

- 确保在其他包中正确引用此包定义的消息类型。
