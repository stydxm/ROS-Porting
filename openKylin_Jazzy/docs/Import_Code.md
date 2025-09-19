# 导入源码

将ROS官方的源码仓库下载到本地，按openKylin的要求上传至它的gitee组织下

## 获取源码

这里以ROS Core为例，REP2001中[^1]定义了ros-jazzy-core应包含的软件包

[^1]: https://www.ros.org/reps/rep-2001.html#jazzy-jalisco-may-2024-may-2029

先安装ROS提供的工具rosinstall[^2]

[^2]: 在Ubuntu22.04中有效，但24.04等更新的Ubuntu版本中移除了这些包。这似乎是因为使用了Python3.4中废弃3.12后彻底移除的imp库。需要使用3.11及以前的Python版本并从PyPI安装

```bash
sudo apt install python3-rosinstall-generator python3-wstool
```

使用rosinstall生成源码列表并使用wstool拉取git仓库

```bash
rosinstall_generator ros_core --rosdistro jazzy --deps > jazzy-core.rosinstall
wstool init src jazzy-core.rosinstall
```

## 生成软件包列表

按oK的要求[^3]，上传源码前要先登记包名

[^3]: https://gitee.com/openkylin/docs/blob/master/04_%E7%A4%BE%E5%8C%BA%E8%B4%A1%E7%8C%AE/%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97/%E7%A7%BB%E6%A4%8DGNU%20Hello%E8%BD%AF%E4%BB%B6%E5%88%B0openKylin.md#3-%E4%BD%9C%E4%B8%BAsig%E7%BB%84%E7%BB%B4%E6%8A%A4%E8%80%85

ROS的包数量多，全部人工写有点麻烦，可以用一小段Python代码来完成

```python
import yaml
import os

data = yaml.safe_load(open("./jazzy-core.rosinstall"))
for item in data:
    local_name = item['git']['local-name']
    last_segment = local_name.split('/')[-1]
    processed_name = last_segment.replace('_', '-')
    print("- ros-jazzy-"+processed_name)
```

<details>
<summary>包列表</summary>

```yaml
- ros-jazzy-ament-cmake
- ros-jazzy-ament-cmake-auto
- ros-jazzy-ament-cmake-core
- ros-jazzy-ament-cmake-export-definitions
- ros-jazzy-ament-cmake-export-dependencies
- ros-jazzy-ament-cmake-export-include-directories
- ros-jazzy-ament-cmake-export-interfaces
- ros-jazzy-ament-cmake-export-libraries
- ros-jazzy-ament-cmake-export-link-flags
- ros-jazzy-ament-cmake-export-targets
- ros-jazzy-ament-cmake-gen-version-h
- ros-jazzy-ament-cmake-gmock
- ros-jazzy-ament-cmake-google-benchmark
- ros-jazzy-ament-cmake-gtest
- ros-jazzy-ament-cmake-include-directories
- ros-jazzy-ament-cmake-libraries
- ros-jazzy-ament-cmake-pytest
- ros-jazzy-ament-cmake-python
- ros-jazzy-ament-cmake-target-dependencies
- ros-jazzy-ament-cmake-test
- ros-jazzy-ament-cmake-vendor-package
- ros-jazzy-ament-cmake-version
- ros-jazzy-ament-cmake-ros
- ros-jazzy-domain-coordinator
- ros-jazzy-ament-index-cpp
- ros-jazzy-ament-index-python
- ros-jazzy-ament-cmake-copyright
- ros-jazzy-ament-cmake-cppcheck
- ros-jazzy-ament-cmake-cpplint
- ros-jazzy-ament-cmake-flake8
- ros-jazzy-ament-cmake-lint-cmake
- ros-jazzy-ament-cmake-pep257
- ros-jazzy-ament-cmake-uncrustify
- ros-jazzy-ament-cmake-xmllint
- ros-jazzy-ament-copyright
- ros-jazzy-ament-cppcheck
- ros-jazzy-ament-cpplint
- ros-jazzy-ament-flake8
- ros-jazzy-ament-lint
- ros-jazzy-ament-lint-auto
- ros-jazzy-ament-lint-cmake
- ros-jazzy-ament-lint-common
- ros-jazzy-ament-mypy
- ros-jazzy-ament-pep257
- ros-jazzy-ament-pycodestyle
- ros-jazzy-ament-uncrustify
- ros-jazzy-ament-xmllint
- ros-jazzy-ament-package
- ros-jazzy-class-loader
- ros-jazzy-actionlib-msgs
- ros-jazzy-common-interfaces
- ros-jazzy-diagnostic-msgs
- ros-jazzy-geometry-msgs
- ros-jazzy-nav-msgs
- ros-jazzy-sensor-msgs
- ros-jazzy-shape-msgs
- ros-jazzy-std-msgs
- ros-jazzy-std-srvs
- ros-jazzy-stereo-msgs
- ros-jazzy-trajectory-msgs
- ros-jazzy-visualization-msgs
- ros-jazzy-console-bridge-vendor
- ros-jazzy-cyclonedds
- ros-jazzy-fastcdr
- ros-jazzy-fastrtps
- ros-jazzy-foonathan-memory-vendor
- ros-jazzy-google-benchmark-vendor
- ros-jazzy-gmock-vendor
- ros-jazzy-gtest-vendor
- ros-jazzy-iceoryx-binding-c
- ros-jazzy-iceoryx-hoofs
- ros-jazzy-iceoryx-posh
- ros-jazzy-launch
- ros-jazzy-launch-testing
- ros-jazzy-launch-testing-ament-cmake
- ros-jazzy-launch-xml
- ros-jazzy-launch-yaml
- ros-jazzy-launch-ros
- ros-jazzy-launch-testing-ros
- ros-jazzy-ros2launch
- ros-jazzy-libstatistics-collector
- ros-jazzy-libyaml-vendor
- ros-jazzy-mimick-vendor
- ros-jazzy-osrf-pycommon
- ros-jazzy-osrf-testing-tools-cpp
- ros-jazzy-performance-test-fixture
- ros-jazzy-pluginlib
- ros-jazzy-pybind11-vendor
- ros-jazzy-python-cmake-module
- ros-jazzy-rcl
- ros-jazzy-rcl-action
- ros-jazzy-rcl-lifecycle
- ros-jazzy-rcl-yaml-param-parser
- ros-jazzy-action-msgs
- ros-jazzy-builtin-interfaces
- ros-jazzy-composition-interfaces
- ros-jazzy-lifecycle-msgs
- ros-jazzy-rcl-interfaces
- ros-jazzy-rosgraph-msgs
- ros-jazzy-service-msgs
- ros-jazzy-statistics-msgs
- ros-jazzy-test-msgs
- ros-jazzy-type-description-interfaces
- ros-jazzy-rcl-logging-interface
- ros-jazzy-rcl-logging-spdlog
- ros-jazzy-rclcpp
- ros-jazzy-rclcpp-action
- ros-jazzy-rclcpp-components
- ros-jazzy-rclcpp-lifecycle
- ros-jazzy-rclpy
- ros-jazzy-rcpputils
- ros-jazzy-rcutils
- ros-jazzy-rmw
- ros-jazzy-rmw-implementation-cmake
- ros-jazzy-rmw-connextdds
- ros-jazzy-rmw-connextdds-common
- ros-jazzy-rti-connext-dds-cmake-module
- ros-jazzy-rmw-cyclonedds-cpp
- ros-jazzy-rmw-dds-common
- ros-jazzy-rmw-fastrtps-cpp
- ros-jazzy-rmw-fastrtps-dynamic-cpp
- ros-jazzy-rmw-fastrtps-shared-cpp
- ros-jazzy-rmw-implementation
- ros-jazzy-tracetools
- ros-jazzy-ros2action
- ros-jazzy-ros2cli
- ros-jazzy-ros2cli-test-interfaces
- ros-jazzy-ros2component
- ros-jazzy-ros2doctor
- ros-jazzy-ros2interface
- ros-jazzy-ros2lifecycle
- ros-jazzy-ros2lifecycle-test-fixtures
- ros-jazzy-ros2multicast
- ros-jazzy-ros2node
- ros-jazzy-ros2param
- ros-jazzy-ros2pkg
- ros-jazzy-ros2run
- ros-jazzy-ros2service
- ros-jazzy-ros2topic
- ros-jazzy-ros2cli-common-extensions
- ros-jazzy-ros-environment
- ros-jazzy-ros2test
- ros-jazzy-ros-testing
- ros-jazzy-rosidl-adapter
- ros-jazzy-rosidl-cli
- ros-jazzy-rosidl-cmake
- ros-jazzy-rosidl-generator-c
- ros-jazzy-rosidl-generator-cpp
- ros-jazzy-rosidl-generator-type-description
- ros-jazzy-rosidl-parser
- ros-jazzy-rosidl-pycommon
- ros-jazzy-rosidl-runtime-c
- ros-jazzy-rosidl-runtime-cpp
- ros-jazzy-rosidl-typesupport-interface
- ros-jazzy-rosidl-typesupport-introspection-c
- ros-jazzy-rosidl-typesupport-introspection-cpp
- ros-jazzy-rosidl-core-generators
- ros-jazzy-rosidl-core-runtime
- ros-jazzy-rosidl-default-generators
- ros-jazzy-rosidl-default-runtime
- ros-jazzy-rosidl-dynamic-typesupport
- ros-jazzy-rosidl-dynamic-typesupport-fastrtps
- ros-jazzy-rosidl-generator-py
- ros-jazzy-rosidl-runtime-py
- ros-jazzy-rosidl-typesupport-c
- ros-jazzy-rosidl-typesupport-cpp
- ros-jazzy-fastrtps-cmake-module
- ros-jazzy-rosidl-typesupport-fastrtps-c
- ros-jazzy-rosidl-typesupport-fastrtps-cpp
- ros-jazzy-rpyutils
- ros-jazzy-spdlog-vendor
- ros-jazzy-sros2
- ros-jazzy-sros2-cmake
- ros-jazzy-test-interface-files
- ros-jazzy-tinyxml2-vendor
- ros-jazzy-uncrustify-vendor
- ros-jazzy-unique-identifier-msgs
- ros-jazzy-ros-core
```
</details>

然后从中手动处理一些特殊的包，比如variants改为ros-jazzy-core

## 生成debian目录

使用apt（或者从PyPI）安装bloom

```bash
sudo apt install python3-bloom
```

设置环境变量[^4]让rosdep将oK V2.0 Nile认为是Ubuntu 24.04 Noble

[^4]: https://wiki.ros.org/ROS/EnvironmentVariables#ROS_OS_OVERRIDE:~:text=that%20language's%20bindings.-,ROS_OS_OVERRIDE,-Format%3A%20%22OS_NAME%3AOS_VERSION_STRING

```bash
export ROS_OS_OVERRIDE=ubuntu:24.04:noble
```

<details>
<summary>加速rosdep下载</summary>

```bash
# 使用以下步骤替代 rosdep init
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.ustc.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list
sed -i 's#raw.githubusercontent.com/ros/rosdistro/master#mirrors.ustc.edu.cn/rosdistro#g' /etc/ros/rosdep/sources.list.d/20-default.list

# 更换源
export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
rosdep update

# 可以考虑持久化以上环境变量：
echo 'export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
```
</details>

执行生成

```bash
bloom-generate rosdebian . --os-name ubuntu --os-version noble --ros-distro jazzy
```

因软件包名导致的依赖问题需要在`debian/control`中手动处理（暂未观测到）

再手动处理一些软件包，例如元包不需要编译，打出来的包不需要区分架构，可以将`debian/control`中的`Architecture`设为`all`

## 测试编译

```bash
dpkg-buildpackage -b -us
```

如果没有别的问题，一段时间后会在上级目录生成出ros-jazzy-xxx.deb
