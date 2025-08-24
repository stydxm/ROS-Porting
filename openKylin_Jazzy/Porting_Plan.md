本方案聚焦于达成如下目标：

**在 openkylin 2.0 sp1 x86 上移植 ROS Jazzy Desktop 变体**

## rosdep 说明

### 验证

更进一步的调研，根据这一[部分]([https://wiki.ros.org/ROS/EnvironmentVariables?utm_source=chatgpt.com#ROS_OS_OVERRIDE:~:text=that language's bindings.-,ROS_OS_OVERRIDE,-Format%3A "OS_NAME%3AOS_VERSION_STRING](https://wiki.ros.org/ROS/EnvironmentVariables?utm_source=chatgpt.com#ROS_OS_OVERRIDE:~:text=that%20language%27s%20bindings.-,ROS_OS_OVERRIDE,-Format%3A%20%22OS_NAME%3AOS_VERSION_STRING))的内容我们可以知道说，在 ROS 非官方支持的发行版中使用 ROS 官方的工具链我们可以尝试使用：`export ROS_OS_OVERRIDE=` 来让 openkylin 伪装成 ubuntu，这样或许可以尝试避免在 openeuler 上那样去移植 rosdep 的问题。

我们已经成功的验证可以通过指定 `ROS_OS_OVERRIDE` 该变量去实现让系统伪装成 ubuntu：

```jsx
export ROS_OS_OVERRIDE=ubuntu:24.04:noble
```

### 存在的问题

但是这么做的话，会使得 rosdep 会把没有带 t64 的包认成缺包实际上已经安装了，这个问题暂时没有想好要怎么解决：

```jsx
executing command [sudo -H apt-get install -y libqt5core5t64]
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成
E: 无法定位软件包 libqt5core5t64
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y libqt5core5t64] failed
```

## 构建方案

当我们解决完了缺失的系统级依赖软件包后，我们需要做的事情是调研一下 Debian 系的软件包要如何进行构建，以及对应在 ROS 上要怎么做。

短期内的构建目标是：ROS core 变体，[即]([https://www.ros.org/reps/rep-2001.html#jazzy-jalisco-may-2024-may-2029:~:text=2024 - May 2029](https://www.ros.org/reps/rep-2001.html#jazzy-jalisco-may-2024-may-2029:~:text=2024%20%2D%20May%202029))-,ROS%20Core,-%2D%20ros_core%3A%0A%20%20%20%20packages%3A%20%5Bament_cmake)：

```jsx
- ros_core:
    packages: [ament_cmake, ament_cmake_auto, ament_cmake_gmock,
               ament_cmake_gtest, ament_cmake_pytest,
               ament_cmake_ros, ament_index_cpp,
               ament_index_python, ament_lint_auto,
               ament_lint_common, class_loader, common_interfaces,
               launch, launch_ros, launch_testing,
               launch_testing_ament_cmake, launch_testing_ros,
               launch_xml, launch_yaml, pluginlib, rcl_lifecycle,
               rclcpp, rclcpp_action, rclcpp_lifecycle, rclpy,
               ros2cli_common_extensions, ros2launch,
               ros_environment, rosidl_default_generators,
               rosidl_default_runtime, sros2, sros2_cmake]
    And at least one of the following rmw_implementation:
    - Fast-RTPS: [Fast-CDR, Fast-RTPS, rmw_fastrtps]
    - CycloneDDS: [cyclonedds, rmw_cyclonedds]
    - Connext: [rmw_connextdds]
```

所以目前的任务是：

- 验证上述 export 指令是否可行

一旦我们确定上述方案可行后，我们直接根据官方给的方案进行构建即可。

但是我们目前还没有确定的内容是我们在本地要如何进行构建，以及我们要如何去进行发布软件包。

暂定的方案是些脚本（大体的思路为：使用 bloom 生成打包需要的前置内容，然后用 dpkg 去进行打包）去进行构建软件包，然后测试这些构建成功的软件包能否正常的安装。

## 移植思路

目前希望用官方工具链来快速的验证软件包的构建正确性。

拉取源码清单：rosinstall

依赖验证：rosdep

构建工具：colcon

源码拉取工具：vcstool

由于官方工具链全部使用 python 编写，所以使用 pipx 进行环境隔离：

```jsx
sudo apt update
sudo apt install -y pipx
pipx ensurepath  # 退出重登或执行 `source ~/.profile` 让 PATH 生效
pipx install rosdep
pipx install rosinstall
pipx install vcstool
```

### 中间件说明

关于中间件，ROS 默认是使用 `rti-connext-dds-6.0.1` ，这是一个商业软件，所以我们改用开源的：rmw_cyclonedds_cpp

所以所有变体构建我们都需要提前 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 

同时，需要在源码中手动移除：`src/rmw_connextdds`

否则会遇到如下的构建错误：

```jsx
Failed to find:
- install/share/rmw_connextdds_common/package.sh
- install/share/rmw_connextdds/package.sh
Check that these packages have been built:
- rmw_connextdds_common
- rmw_connextdds
```

## 移植过程

### ros_core & ros_base

首先，在 ～ 下创建 ros2_wk

创建源码清单，该过程会调用 rosinstall 将 ros_core 相关的软件包源码清单进行下载保存为  ros_core.repo：

```jsx
rosinstall_generator ros_core --rosdistro jazzy --deps > ros_core.repo
```

进入 ros2_wk 文件夹后，使用 vcs 工具拉取源码：

```jsx
vcs import src < ~/ros_core.repo
```

使用 rosdep 去预先安装需要的依赖：

```jsx
 rosdep install -y --rosdistro jazzy --from-paths src --ignore-src
```

在这过程中需要特殊说明的是，存在许多的相关 python 软件包缺包，这一部后续搭建起来构建平台后可以我们自己来实现，缺包清单见这里（）：

```jsx
rosdep install -y --rosdistro jazzy --from-paths src --ignore-src
--skip-keys="python3-vcstool python3-catkin-pkg-modules rti-connext-dds-6.0.1 python3-rosdistro-modules python3-mypy ros-jazzy-rmw-connextdds"
```

其他说明：实际上在构建的过程中还缺少了如下的包：

- numpy
- pytest
- typing-extensions
- empy
- numpy
- pyparsing
- lark
- lark-parser

开始构建：

```jsx
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

colcon build \
  --merge-install --symlink-install \
  --parallel-workers "$(nproc)" \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$COLCON_PY" \
  --packages-up-to ros_core \
  --packages-skip rmw_connextdds rmw_connextdds_common
```

构建结果：

```jsx
Summary: 175 packages finished [3min 37s]
  41 packages had stderr output: ament_copyright ament_cppcheck ament_cpplint ament_flake8 ament_index_python ament_lint ament_lint_cmake ament_mypy ament_package ament_pep257 ament_pycodestyle ament_uncrustify ament_xmllint domain_coordinator launch launch_ros launch_testing launch_testing_ros launch_xml launch_yaml osrf_pycommon ros2action ros2cli ros2component ros2doctor ros2interface ros2launch ros2lifecycle ros2multicast ros2node ros2param ros2pkg ros2run ros2service ros2test ros2topic rosidl_cli rosidl_pycommon rosidl_runtime_py rpyutils sros2
➜  ros2_ws

```

对于 ros base 的过程也是一致的，不同的地方是：

- 源码拉取的时候请写 ros_base
- 需要 rosdep 跳过的包不一样
    
    ```jsx
    rosdep install -y --rosdistro jazzy --from-paths src --ignore-src \
      --skip-keys="rmw_connextdds rmw_connextdds_common rti-connext-dds-6.0.1 \
                   rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp \
                   connext_cmake_module rti_connext_dds_cmake_module python3-vcstool python3-catkin-pkg-modules rti-connext-dds-6.0.1 python3-rosdistro-modules python3-mypy"
    ```
    

ros base 预期的构建结果：

```jsx
Finished <<< ros_base [0.85s]

Summary: 223 packages finished [3min 5s]
  67 packages had stderr output: ament_clang_format ament_copyright ament_cppcheck ament_cpplint ament_flake8 ament_index_python ament_lint ament_lint_cmake ament_mypy ament_package ament_pep257 ament_pycodestyle ament_uncrustify ament_xmllint common_interfaces domain_coordinator geometry2 kdl_parser launch launch_ros launch_testing launch_testing_ros launch_xml launch_yaml osrf_pycommon python_orocos_kdl_vendor ros2action ros2bag ros2cli ros2cli_common_extensions ros2component ros2doctor ros2interface ros2launch ros2lifecycle ros2multicast ros2node ros2param ros2pkg ros2run ros2service ros2test ros2topic ros_base ros_core ros_testing rosbag2 rosbag2_interfaces rosbag2_storage_default_plugins rosbag2_test_msgdefs rosbag2_tests rosidl_cli rosidl_pycommon rosidl_runtime_py rpyutils sensor_msgs_py sros2 sros2_cmake stereo_msgs tf2 tf2_eigen_kdl tf2_msgs tf2_py tf2_ros_py tf2_tools urdf visualization_msgs
➜  ros2_ws
```

### Desktop

这个变体中[预期](https://www.ros.org/reps/rep-2001.html#id98)新增的软件包是：

```jsx
Desktop
- desktop:
    extends:  [ros_base]
    packages: [action_tutorials_cpp, action_tutorials_interfaces,
               action_tutorials_py, angles, composition,
               demo_nodes_cpp, demo_nodes_cpp_native,
               demo_nodes_py, depthimage_to_laserscan,
               dummy_map_server, dummy_robot_bringup,
               dummy_sensors,
               examples_rclcpp_minimal_action_client,
               examples_rclcpp_minimal_action_server,
               examples_rclcpp_minimal_client,
               examples_rclcpp_minimal_composition,
               examples_rclcpp_minimal_publisher,
               examples_rclcpp_minimal_service,
               examples_rclcpp_minimal_subscriber,
               examples_rclcpp_minimal_timer,
               examples_rclcpp_multithreaded_executor,
               examples_rclpy_executors,
               examples_rclpy_minimal_action_client,
               examples_rclpy_minimal_action_server,
               examples_rclpy_minimal_client,
               examples_rclpy_minimal_publisher,
               examples_rclpy_minimal_service,
               examples_rclpy_minimal_subscriber, image_tools,
               intra_process_demo, joy, lifecycle, logging_demo,
               pcl_conversions, pendulum_control, pendulum_msgs,
               quality_of_service_demo_cpp,
               quality_of_service_demo_py, rqt_common_plugins,
               rviz2, rviz_default_plugins, teleop_twist_joy,
               teleop_twist_keyboard, tlsf, tlsf_cpp,
               topic_monitor, turtlesim]
```

所以这个变体能起来小乌龟也能起来了。

目前已知在移植 desktop 时存在的问题是：

1. rosinstall 无法下载 desktop 变体
2. 存在 t64 包命名问题
3. openkylin 上的包版本不满足 jazzy 要求

关于 rosinstall 的问题是：

```jsx
➜  ~ rosinstall_generator ros_desktop --rosdistro jazzy --deps > ~/ros_desktop.repos
The following unreleased packages/stacks will be ignored: ros_desktop
No packages/stacks left after ignoring unreleased
➜  ~
```

目前暂时的解决方案是自己[根据](https://www.ros.org/reps/rep-2001.html#id40:~:text=packages%3A%20%5Bgeometry2%2C%20kdl_parser%2C%20robot_state_publisher%2C%0A%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20rosbag2%2C%20urdf%5D-,Desktop,-%2D%20desktop%3A%0A%20%20%20%20extends%3A%20%20%5Bros_base%5D%0A%20%20%20%20packages%3A%20%5Baction_tutorials_cpp%2C%20action_tutorials_py)官网上的包去自己一个一个的找，但是这样存在分支问题，需要单独找时间我们自己维护一份 `desktop.repo` 。

关于 t64 包的问题是，因为 rosdep 返回如下：

```jsx
executing command [sudo -H apt-get install -y libqt5core5t64]
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成
E: 无法定位软件包 libqt5core5t64
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y libqt5core5t64] failed
```

目前考虑直接让 rosdep 跳过。

关于 openkylin 上依赖版本不满足的问题是：

```jsx

executing command [sudo -H apt-get install -y libpcl-features1.14]
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成
E: 无法定位软件包 libpcl-features1.14
E: 无法按照 glob ‘libpcl-features1.14’ 找到任何软件包
E: 无法按照正则表达式 libpcl-features1.14 找到任何软件包
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y libpcl-features1.14] failed
➜  ros2_ws apt search libpcl-features
正在排序... 完成
全文搜索... 完成
libpcl-features1.13/nile,now 1.13.0+dfsg-ok1 amd64 [已安装，自动]
  Point Cloud Library - features library

➜  ros2_ws
```

关于这部分的内容是我们需要考虑我们要自己去调研一下这个问题是怎么一回事。

## 缺包清单

### ros_core

```jsx
"catkin_pkg>=0.5.2"
 "rosdistro>=0.9,<1.0"
 "mypy>=1.9.0"
 python3-vcstool
 numpy
 pytest
 typing-extensions
 empy
 numpy 
 pyparsing
 lark lark-parser
```