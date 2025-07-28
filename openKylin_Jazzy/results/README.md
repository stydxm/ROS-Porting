# openKylin 主要问题

1. 存在大量无法安装的包，如：

```
openkylin@openkylin:~$ sudo apt install liburcu-dev -y
您也许需要运行“apt --fix-broken install”来修正上面的错误。
无法满足的依赖关系：
 libcanberra-pulse : 依赖: pulseaudio 但是它将不会被安装
 liburcu-dev : 依赖: liburcu8t64 (= 0.14.0-ok3) 但是它将不会被安装
 parchives : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
             推荐: engrampa 但是它将不会被安装
 peony-bluetooth : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-device-rename : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-extension-computer-view : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-open-terminal : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-send-to-device : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-share : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 pulseaudio-module-bluetooth : 依赖: pulseaudio (= 1:16.1+dfsg1-ok1) 但是它将不会被安装
                               推荐: gstreamer1.0-plugins-bad (>= 1.19) 但是它将不会被安装
 ukui-bluetooth : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 ukui-globaltheme-heyin : 依赖: qt5-ukui-platformtheme (> 3.1.0) 但是它将不会被安装
错误： 有未能满足的依赖关系。请尝试不指明软件包的名字来运行“apt --fix-broken install”(也可以指定一个解决办法)。
错误： The following information from --solver 3.0 may provide additional context:
   有未能满足的依赖关系。请尝试不指明软件包的名字来运行“apt --fix-broken install”(也可以指定一个解决办法)。
openkylin@openkylin:~$ sudo apt install pydocstyle -y
您也许需要运行“apt --fix-broken install”来修正上面的错误。
无法满足的依赖关系：
 libcanberra-pulse : 依赖: pulseaudio 但是它将不会被安装
 parchives : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
             推荐: engrampa 但是它将不会被安装
 peony-bluetooth : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-device-rename : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-extension-computer-view : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-open-terminal : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-send-to-device : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 peony-share : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 pulseaudio-module-bluetooth : 依赖: pulseaudio (= 1:16.1+dfsg1-ok1) 但是它将不会被安装
                               推荐: gstreamer1.0-plugins-bad (>= 1.19) 但是它将不会被安装
 pydocstyle : 依赖: python3-pydocstyle (= 6.3.0-ok1) 但是它将不会被安装
 ukui-bluetooth : 依赖: libpeony3 (>= 3.0.2) 但是它将不会被安装
 ukui-globaltheme-heyin : 依赖: qt5-ukui-platformtheme (> 3.1.0) 但是它将不会被安装
错误： 有未能满足的依赖关系。请尝试不指明软件包的名字来运行“apt --fix-broken install”(也可以指定一个解决办法)。
错误： The following information from --solver 3.0 may provide additional context:
   有未能满足的依赖关系。请尝试不指明软件包的名字来运行“apt --fix-broken install”(也可以指定一个解决办法)。
```

2. 缺包

目前根据该目录中的工作流已经成功验证出缺包清单，具体缺包如下：

- python3-supported-min
- python3-rospkg-modules
- python3-rosdistro-modules
- python3-cffi-backend-api-min
- python3-cffi-backend-api-max
- python3-catkin-pkg-modules
- debconf-2.0
- libcurl4t64
- libhogweed6
- liblttng-ctl0t64
- liblttng-ust-common1t64
- liblttng-ust-ctl5t64
- liblttng-ust-python-agent1t64
- liblttng-ust1t64
- libreadline8t64
- libspdlog1.12-fmt9


