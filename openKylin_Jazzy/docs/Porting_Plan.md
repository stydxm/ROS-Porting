# openkylin 方案说明

# 背景

本方案聚焦于在 openkylin 2.0 sp1 x86 上移植 ROS Jazzy Desktop 变体。主要是说明如何通过 ROS 官方工具链去对软件包进行构建。

本方案分为两个部分， 第一个部分是使用 colcon 去进行软件包构建正确性的验证，另一部分是自动化构建 deb 软件包以及发布。

本文主要是记录我们遇到问题以及如何解决以及移植的全过程。

# Colcon 验证

我们的验证思路是根据官方给的[变体](https://www.ros.org/reps/rep-2001.html#jazzy-jalisco-may-2024-may-2029:~:text=perception%2C%20simulation%2C%20ros_ign_gazebo_demos%5D-,Jazzy%20Jalisco%20(May%202024%20%2D%20May%202029),-ROS%20Core)，也就是如下变体：

- [Jazzy Jalisco (May 2024 - May 2029)](https://www.ros.org/reps/rep-2001.html#jazzy-jalisco-may-2024-may-2029)
    - [ROS Core](https://www.ros.org/reps/rep-2001.html#id32)
    - [ROS Base](https://www.ros.org/reps/rep-2001.html#id33)
    - [Desktop](https://www.ros.org/reps/rep-2001.html#id34)
    - [Perception](https://www.ros.org/reps/rep-2001.html#id35)
    - [Simulation](https://www.ros.org/reps/rep-2001.html#id36)
    - [Desktop Full](https://www.ros.org/reps/rep-2001.html#id37)

我们决定从最基本的 ROS Core 开始去进行移植验证，再一步步的推进到 Desktop。

### 源码

在准备 ros_core 以及 ros_base 的源码的时候都是很顺利的使用 rosinstall 去进行获取的。也就是：

```jsx
rosinstall_generator ros_core --rosdistro jazzy --deps > ros_core.repo
```

然后使用 vcstool 去拉取源码到 src 文件夹下面：

```jsx
vcs import src < ~/ros_core.repo
```

对于 ros_base 也是一样的道理，就是改一下指令：

```jsx
rosinstall_generator ros_base --rosdistro jazzy --deps > ros_base.repo
vcs import src < ~/ros_base.repo
```

但是到了 desktop 之后就会遇到：

```jsx
➜  ~ rosinstall_generator ros_desktop --rosdistro jazzy --deps > ~/ros_desktop.repos
The following unreleased packages/stacks will be ignored: ros_desktop
No packages/stacks left after ignoring unreleased
➜  ~
```

rosinstall 本来也就不是为了 ROS2 准备的，而 ROS 官方维护的 desktop 的源码清单是：`ros2.repo`

也就是该[仓库](https://github.com/ros2/ros2/tree/jazzy)下的文件。

所以对于 desktop 的源码获取需要：

```jsx
vcs import src < ~/ros2.repo
```

现在源码准备完了，我们就遇到了第一个问题：ROS 官方工具链并没有对 openkylin 进行支持。

## rosdep

所以一开始的思路是去移植工具，不过后来随着对官方工具链调研的深入，发现https://wiki.ros.org/ROS/EnvironmentVariables?utm_source=chatgpt.com#ROS_OS_OVERRIDE:~:text=that%20language%27s%20bindings.-,ROS_OS_OVERRIDE,-Format%3A%20%22OS_NAME%3AOS_VERSION_STRING了：

> **ROS_OS_OVERRIDE**
> 
> 
> Format: "OS_NAME:OS_VERSION_STRING:OS_CODENAME" This will force it to detect Ubuntu Bionic:
> 
> ```
> export ROS_OS_OVERRIDE=ubuntu:18.04:bionic
> ```
> 
> If defined, this will override the autodetection of an OS. This can be useful when debugging rosdep dependencies on alien platforms, when platforms are actually very similar and might need be forced, or of course if the autodetection is failing.
> 

也就是说，我们可以通过 `export ROS_OS_OVERRIDE` 这个变量去让 openkylin 伪装成 ubuntu，既然 openkylin 是基于 ubuntu，那就这么做理论上确实也可以。

在后续我们也确实是验证了这个参数是可行的：

```jsx
(.venv) ➜  ros2_ws export ROS_OS_OVERRIDE=ubuntu:24.04:noble

rosdep install -y --rosdistro jazzy --from-paths src --ignore-src \
  --skip-keys="python3-vcstool python3-catkin-pkg-modules rti-connext-dds-6.0.1 python3-rosdistro-modules python3-mypy"
executing command [sudo -H apt-get install -y libncurses-dev]
[sudo] test1 的密码：
```

rosdep 如预期的开始使用 apt 去安装依赖。

这么伪装大体上是没问题的，但是也只是大体，还是有一些小问题的。

### t64命名问题

这么做的话，会使得 rosdep 会把没有带 t64 的包认成缺包实际上已经安装了：

```jsx
executing command [sudo -H apt-get install -y libqt5core5t64]
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成
E: 无法定位软件包 libqt5core5t64
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y libqt5core5t64] failed
```

这个问题其实也很好解决，就是通过 `--skip-keys` 这个参数告诉 rosdep 跳过这个软件包就行，回头自己手动装上。

但是这样会衍生出一个新的问题，在后面 bloom 去写依赖的时候肯定是按照 t64 的包去写。这是一个问题，要么改 rosdep yaml 里的 key，要么手动改依赖名称。不过到时候构建的时候再看吧（todo）。

构建到 desktop 的时候我们需要跳过的 key 有下面这么多，都是一点一点尝试出来的，跳过也是各有各的原因：

```jsx

rosdep install -y --rosdistro jazzy --from-paths src --ignore-src \
  --skip-keys="\
rmw_connextdds \
rmw_connextdds_common \
rti-connext-dds-6.0.1 \
rosidl_typesupport_connext_c \
rosidl_typesupport_connext_cpp \
connext_cmake_module \
rti_connext_dds_cmake_module \
python3-vcstool \
python3-catkin-pkg-modules \
python3-rosdistro-modules \
python3-mypy \
ros-jazzy-example-interfaces \
ros-jazzy-gz-math-vendor \
ros-jazzy-diagnostic-updater \
ros-jazzy-rqt-action \
ros-jazzy-pcl-msgs \
libpcl-common \
ros-jazzy-resource-retriever \
libpcl-features \
libpcl-io"
```

一部分一部分的说明，关于一些依赖是因为版本的问题：

```jsx
ros-jazzy-resource-retriever \
libpcl-features"
executing command [sudo -H apt-get install -y libpcl-io1.14]
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成
E: 无法定位软件包 libpcl-io1.14
E: 无法按照 glob ‘libpcl-io1.14’ 找到任何软件包
E: 无法按照正则表达式 libpcl-io1.14 找到任何软件包
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y libpcl-io1.14] failed
```

实际上是有 1.13 版本的：

```jsx
(.venv) ➜  ros2_ws apt search libpcl-io
正在排序... 完成
全文搜索... 完成
libpcl-io1.13/nile,now 1.13.0+dfsg-ok1 amd64 [已安装]
  Point Cloud Library - I/O library
```

版本并不会影响到构建，所以也是跳过自己手动装就行。

至于 ros-jazzy 开头的就不用说了，那都是后面需要构建出来的依赖，不知道为什么被 rosdep 读出来但是没有跳过。

### 中间件问题

关于 rmw，也就是中间件的情况是，由于 ROS2 官方使用的是商业软件，所以我们决定使用开源的中间件，所以跳过，也就是这一部分：

```jsx
rmw_connextdds \
rmw_connextdds_common \
rti-connext-dds-6.0.1 \
```

这么跳过没问题，但是需要注意：

1. 在构建前，需要提前指定中间件：`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 
2. 需要将 rmw_connextdds 从 src 中移除：`mv **src/rmw_connextdds** ~`

如果你不移除会遇到如下问题（即使你指定了）：

```jsx
Failed to find:
- install/share/rmw_connextdds_common/package.sh
- install/share/rmw_connextdds/package.sh
Check that these packages have been built:
- rmw_connextdds_common
- rmw_connextdds
```

剩下的就是关于 python 相关的软件包了。

### 缺少的 python 包

在 openkylin 上缺失 ROS2 构建时需要的 python 库

```jsx
Traceback (most recent call last):
  File "/home/test1/ros2_ws/src/ament/ament_cmake/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 22, in <module>
    from catkin_pkg.package import parse_package_string
ModuleNotFoundError: No module named 'catkin_pkg'
CMake Error at cmake/core/ament_package_xml.cmake:95 (message):
  execute_process(/usr/bin/python3
  /home/test1/ros2_ws/src/ament/ament_cmake/ament_cmake_core/cmake/core/package_xml_2_cmake.py
  /home/test1/ros2_ws/src/ament/ament_cmake/ament_cmake_core/package.xml
  /home/test1/ros2_ws/build/ament_cmake_core/ament_cmake_core/package.cmake)
  returned error code 1
Call Stack (most recent call first):
  cmake/core/ament_package_xml.cmake:49 (_ament_package_xml)
  CMakeLists.txt:15 (ament_package_xml)

```

尝试过使用 apt 进行前置依赖补全，但是发现还是缺失部分的依赖，如：

```jsx

(.venv) ➜  ros2_ws apt search catkin_pkg
正在排序... 完成
全文搜索... 完成
```

## 前置环境部署

在这样的背景下，我们决定使用虚拟环境验证，直接使用 pip 去安装 ROS2 构建时所需要的包：

```jsx
python3 -m venv .venv

source .venv/bin/activate
python -m pip install -U pip wheel setuptools

pip install catkin_pkg rospkg empy lark pyyaml
```

这么做确实有效，但是要注意在 colcon 构建的参数里面指定虚拟环境的包，也就是增加两个参数：

```jsx
               -DPython3_EXECUTABLE="$PWD/.venv/bin/python" \
               -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python" \
```

## 构建开始

有了源码，依赖也装完了，剩下的也就是构建了。

这个事情说得简单其实也是跑了好十几轮，只是好在我的机器性能够强，速度还挺快，遇到的问题就是一些缺包什么的，然后最后总结出了上面的那些清单。

构建指令：

```jsx
cd ros2_ws

colcon build \
  --merge-install --symlink-install \
  --parallel-workers "$(nproc)" \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release \
               -DPython3_EXECUTABLE="$PWD/.venv/bin/python" \
               -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python" \
  --packages-up-to desktop \
  --packages-skip python_orocos_kdl_vendor rmw_connextdds rmw_connextdds_common

```

### python_orocos_kdl_vendor

这个包一直都是处于一个构建失败的状情况，怀疑是这个包在使用系统的2. **pybind11 头文件**（/usr/include/pybind11/operators.h），与 PyKDL 里绑定的 hash(self) 不兼容。

目前没有解决，但考虑到是个 vendor 包就不是那么紧急，先跳过了。

### 小乌龟启动

总之在使用上面的指令的情况下成功的构建起来 desktop 了，小乌龟也跑起来了。

![alt text](./img/ok_turtlesim.png)

既然小乌龟跑通了，剩下的 deb 包跑通也只是时间问题了。

## 工具

在这个过程中为了解决依赖的问题做了一个简单的[脚本](./tools/scan_rosdep.sh)。

为什么要做？

就是 rosdep 不知道抽什么风，指定跳过的 key 会重复跳出来说找不到，ros-jazzy 相关的包也会跳出来。那在拿工具没办法的情况下，也就只能自己做个简单的工具来避免这些重复耗时耗神的机械工作了。

流程大概就是下面这样：

1. **干跑（simulate）**：通过 `rosdep install --simulate` 获取“计划安装的 apt 命令”。
2. **规范化**：抽取命令中的包名、去重排序，得到“**想安装**”的全集。
3. **仓库对照**：用 `apt-cache pkgnames` 获得“**本地仓库存在**”的包名全集，做集合差异，得到“**缺失**”集合。
4. **分类**：将缺失集合按前缀拆分为：
    - 系统库（非 `ros-` 开头）→ 需要做包名/版本适配；
    - ROS 二进制（`ros-jazzy-*`）→ 通常应从源码构建或加入 `-skip-keys`。
5. **建议生成**：
    - 规则 A：`t64` → 去掉 `t64` 后若存在则建议；
    - 规则 B：带版本后缀（如 `1.14`）→ 在同前缀家族中选择本仓库的**最高可用版本**作为建议；
    - 规则 C：无匹配 → 标注 `(no-suggestion)`，留待手工判断。

跑的流程就是：

```jsx
(.venv) ➜  ros2_ws ./tools/scan_rosdep.sh
[I] 使用 ROS_OS_OVERRIDE=ubuntu:24.04:noble
[1/4] 生成 rosdep 计划安装的 APT 包清单（干跑 simulate）...

[2/4] 提取 apt-get 安装的包名...
[3/4] 和当前 APT 仓库比对，找出缺包...
[4/4] 为缺失的系统库生成重命名/版本候选（t64 / 邻近版本）...

== 结果文件 ==
全部待装包            : /home/test1/ros2_ws/.rosdep_scan/apt_wanted.txt (数量: 37)
缺失包（总）          : /home/test1/ros2_ws/.rosdep_scan/apt_missing_all.txt (数量: 25)
缺失包（系统库）      : /home/test1/ros2_ws/.rosdep_scan/apt_missing_system.txt (数量: 6)
缺失包（ROS 二进制）  : /home/test1/ros2_ws/.rosdep_scan/apt_missing_rosbins.txt (数量: 19)
系统库候选映射        : /home/test1/ros2_ws/.rosdep_scan/apt_missing_suggestions.txt

提示：
  - /home/test1/ros2_ws/.rosdep_scan/apt_missing_rosbins.txt 里的 'ros-jazzy-*' 多半需要改为“源码拉取进 src/”或加到 --skip-keys。
  - 下一步可执行： ./tools/gen_rosdep_overlay.sh  生成本地 rosdep 覆盖 YAML。
```

然后你能查看结果：

```jsx
(.venv) ➜  ros2_ws cd .rosdep_scan
(.venv) ➜  .rosdep_scan cat apt_missing_all.txt
libqt5core5t64
libqt5gui5t64
libqt5widgets5t64
python3-catkin-pkg-modules
python3-rosdistro-modules
python3-vcstool
ros-jazzy-angles
ros-jazzy-depthimage-to-laserscan
ros-jazzy-image-pipeline
ros-jazzy-image-transport-plugins
ros-jazzy-joy
ros-jazzy-laser-filters
ros-jazzy-pcl-conversions
ros-jazzy-perception-pcl
ros-jazzy-rmw-connextdds
ros-jazzy-ros-gz-bridge
ros-jazzy-ros-gz-image
ros-jazzy-ros-gz-interfaces
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-sim-demos
ros-jazzy-rqt-common-plugins
ros-jazzy-teleop-twist-joy
ros-jazzy-teleop-twist-keyboard
ros-jazzy-urdfdom-headers
ros-jazzy-vision-opencv
(.venv) ➜  .rosdep_scan cat apt_missing_system.txt
libqt5core5t64
libqt5gui5t64
libqt5widgets5t64
python3-catkin-pkg-modules
python3-rosdistro-modules
python3-vcstool
```

这么做的好处就是我不需要一遍一遍的跑 rosdep 以及 apt，脚本可以直接帮我把这个事情做了。

我现在就知道我缺少的系统级依赖就是：

```jsx
libqt5core5t64
libqt5gui5t64
libqt5widgets5t64
python3-catkin-pkg-modules
python3-rosdistro-modules
python3-vcstool
```

我们上文说过了，t64 的包不是问题，python 包的缺失也通过虚拟环境解决了，所以我们才可以快速的跑通小乌龟。这就是这个脚本的作用以及意义。