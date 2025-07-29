好的，这是一个为该脚本编写的简单 README。

-----

# OBS SRPM 批量导入脚本

这是一个简单的 Bash 脚本，用于将本地目录中的多个 SRPM (Source RPM) 包批量导入并提交到 Open Build Service (OBS) 项目中。

## 功能

  * 自动遍历指定目录下的所有 SRPM 文件。
  * 将每个 SRPM 导入到 OBS 项目中。
  * 自动将新导入的包 `add` 并 `ci` (commit) 到服务器。
  * 跳过服务器上已经存在的包。
  * 尝试修复并提交之前导入失败（只创建了本地目录但未提交）的包。
  * 记录详细的操作日志和失败列表。

## 解决了什么问题？

1.  **`osc` 缺少批量处理能力**: `osc` 命令行工具本身不支持一次性导入和提交多个 SRPM 包，需要对每个包手动执行 `importsrcpkg`, `add`, `ci` 等多个命令，非常繁琐。这个脚本将整个流程自动化。
2.  **保证上传完整性**: 脚本采用**串行**方式，即一个接一个地处理包。这是为了避免并行处理时可能出现的竞争条件或不完整的文件上传，从而确保每个包都能被完整、正确地提交到 OBS 服务器。

## 如何使用

#### 1\. 准备工作

  * 确保你已经安装并配置好了 `osc` 命令行工具。
  * 使用 `osc` 将你的 OBS 项目 checkout 到本地，例如：
    ```bash
    osc -A https://build.tarsier-infra.isrc.ac.cn co home:Sebastianhayashi:ROS-Humble-openEuler
    ```
  * 将所有需要上传的 `.src.rpm` 文件放在一个单独的目录中（例如 `~/ros_humble_srpm`）。

#### 2\. 配置脚本

打开脚本文件，修改头部的几个变量：

  * `OBS_API`: 你的 OBS 服务 API 地址。
  * `OBS_PROJECT`: 你的 OBS 项目名称。
  * `SRPM_DIR`: 存放 SRPM 文件的目录的**绝对路径**。

#### 3\. 运行脚本

1.  **重要**: 在终端中，**必须**进入你本地的 OBS 项目工作目录。
    ```bash
    # 示例路径，请根据你的实际情况修改
    cd ~/home:Sebastianhayashi:ROS-Humble-openEuler
    ```
2.  将此脚本放在该目录下，并赋予执行权限：
    ```bash
    chmod +x your_script_name.sh
    ```
3.  运行脚本：
    ```bash
    ./your_script_name.sh
    ```

#### 4\. 查看结果

脚本会在运行时实时输出进度。执行完毕后，会生成：

  * `import-fixed-YYYYMMDD-HHMMSS.log`: 详细的操作日志。
  * `failed-imports-YYYYMMDD-HHMMSS.txt`: 记录所有导入失败的 SRPM 包路径，方便后续排查。
