# Odin1 + GENISOM 机器狗 + jie_3d_nav 项目交接文档

> 交接版本：2026-08-26
>
> 项目仓库：`https://github.com/yuanliang111/odin1_jie_nav_ws.git`
>
> 当前正式分支：`main`（仓库当前只保留 `main`）
>
> 当前已验证基线 Commit：`f76814094ed249f12802069d53af4b8aed6a7f18`
>
> Commit 信息：`feat: expose real navigation speed limits`
>
> 本文档目标：让一个此前没有接触过 Odin1、MindCloud、jie_3d_nav 和 GENISOM SDK 的接手人员，从“理解项目是什么”开始，能够完成地图采集、MindCloud 后处理、PCD 抽稀/清理、PCD→OctoMap、Navigation Map Package 生成、地图部署、Odin1 重定位、Web 规划、路径跟踪和机器狗真实平面自主导航。

---

# 1. 先用 5 分钟理解这个项目

## 1.1 项目最终在做什么

整套系统的目标链路是：

```text
Odin1 采集环境
    ↓
OLX 原始记录
    ↓
Windows MindCloud 后处理
    ├── map_merged.bin   → Odin1 重定位
    └── scene_raw.pcd    → 导航地图来源
                              ↓
                        PC 端抽稀/裁剪
                              ↓
                         scene_nav.pcd
                              ↓
                         jie_octomap
                              ↓
                     OctoMap + planner layers
                              ↓
                   Navigation Map Package
                              ↓
          部署 BIN + Map Package 到机器狗
                              ↓
                        Odin1 重定位
                              ↓
                   /odin1/odometry + TF
                              ↓
                        Web 选择目标
                              ↓
                    jie_3d_nav 3D A*
                              ↓
                        /planned_path
                              ↓
                    jie_dog_controller
                              ↓
              /dog/desired_cmd_normalized
                              ↓
                    genisom_state_node
                              ↓
                      GENISOM SDK
                              ↓
                     真实机器狗运动
```

当前项目已经把上述链路在**平面真实机器狗**上完整跑通，包括：

```text
Odin1 重定位                  PASS
OctoMap 自动加载              PASS
Web 地图显示                  PASS
Web 选择导航目标              PASS
jie_3d_nav 路径规划           PASS
/planned_path                 PASS
Odin 位姿闭环跟踪              PASS
原地转向 turn-in-place        PASS
GENISOM SDK 控制              PASS
机器狗真实前进/转向            PASS
到达目标                      PASS
Web Stop 后执行链 DISABLED     PASS
启动时指定前进/转向速度上限      PASS
```

当前阶段**先以平面导航为正式基线**。真正 3D 高差/坡道路径跟踪留作后续扩展。

---

# 2. 一个非常重要的架构原则：采集在机器狗，处理在电脑，再放回机器狗

这是接手人员最容易混淆的地方。

## 2.1 哪些事情在机器狗上做

机器狗负责：

1. Odin1 USB 连接；
2. 启动 Odin ROS Driver；
3. 在实际场地移动机器狗并采集 OLX；
4. 运行正式导航系统；
5. 使用已经处理好的 BIN 完成重定位；
6. 加载已经处理好的 Navigation Map Package；
7. 运行 Web/Planner/Controller/GENISOM bridge；
8. 真实机器狗自主运动。

**不要在机器狗上做大规模地图点云后处理，也不要把机器狗当源码真值来源。**

---

## 2.2 哪些事情在自己的电脑上做

开发电脑负责：

1. Git/GitHub 正式源码；
2. 接收机器狗采集的完整 OLX 目录；
3. MindCloud 后处理（MindCloud 官方流程要求 Windows，因此如果开发电脑是 Ubuntu，可使用双系统/另一台 Windows 电脑）；
4. 将 MindCloud 导出的 BIN/PCD 放回 Ubuntu 开发环境；
5. PCD 抽稀；
6. PCD 手工裁剪/删除明显异常；
7. PCD → OctoMap；
8. 生成 preblocked / traversable / risk cost 等导航层；
9. 保存 Navigation Map Package；
10. 修改项目中的地图路径配置；
11. 编译、代码测试；
12. 再将源码和地图资产部署到机器狗。

原则是：

```text
机器狗 = 采集 + 运行端
PC     = 源码真值 + 地图处理端
GitHub = 代码长期真值
```

---

# 3. 当前硬件、系统、账号与目录

## 3.1 开发电脑

当前已验证开发机：

```text
用户：robot@yuan
系统：Ubuntu 22.04
ROS：ROS 2 Humble
架构：x86_64
```

代码工作空间：

```text
/home/robot/project/odin1_jie_nav_ws
```

地图/数据工作目录：

```text
/home/robot/project/odin1_jie_nav_data
```

---

## 3.2 机器狗

当前最近一次真实运行使用：

```text
SSH：firefly@192.168.0.106
密码：firefly
系统：Ubuntu 22.04
ROS：ROS 2 Humble
架构：aarch64 / ARM64
```

机器狗代码工作空间：

```text
/home/firefly/odin1_jie_nav_ws
```

机器狗地图数据目录：

```text
/home/firefly/odin1_jie_nav_data
```

> 注意：机器狗 IP 可能因为网络环境发生变化。交接后每次第一次工作前，先 `ping` 和 `ssh` 确认，不要只凭文档假设 IP 永远不变。

---

# 4. ROS 运行环境：当前正式基线使用 Domain 77

机器狗正式导航当前统一使用：

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOCALHOST_ONLY=0
```

以后新开的所有正式导航终端，都建议在 `source` 后明确设置这三个环境变量。

不要让 factory ROS 网络和本项目控制网络混在同一个 Domain 中，否则很容易出现 TF 或控制源冲突。

---

# 5. 重要网址与资料

## 5.1 本项目正式仓库

```text
https://github.com/yuanliang111/odin1_jie_nav_ws.git
```

当前仓库只保留：

```text
main
```

当前已验证基线：

```text
f76814094ed249f12802069d53af4b8aed6a7f18
```

---

## 5.2 jie_3d_nav 上游

```text
https://github.com/6-robot/jie_3d_nav.git
```

本项目是基于该项目进行复现和 Odin1/GENISOM 真机适配。

---

## 5.3 Odin1 官方 Wiki

Odin1 用户手册：

```text
https://manifoldtechltd.github.io/wiki/odin_series/odin1/Cover.html
```

Odin1 系列入口：

```text
https://manifoldtechltd.github.io/wiki/odin_series/odin1/
```

建图与重定位建议调用流程：

```text
https://manifoldtechltd.github.io/wiki/odin_series/odin1/6.%20map%20and%20relocalizaiton%20api.html
```

Odin1 Ubuntu 快速启动：

```text
https://manifoldtechltd.github.io/wiki/odin_series/odin1/7.%20Odin1%20Quick%20start%20for%20ubuntu.html
```

重定位功能说明：

```text
https://manifoldtechltd.github.io/wiki/odin_series/odin1/10.%20Relocalization%20Usr%20Guide.html
```

重定位地图获取手册：

```text
https://manifoldtechltd.github.io/wiki/odin_series/odin1/11.%20Relocalization%20Map%20Acquisition%20Guide.html
```

Odin ROS Driver：

```text
https://github.com/manifoldsdk/odin_ros_driver.git
```

---

## 5.4 MindCloud Studio

官方下载入口：

```text
https://version.manifoldtech.cn/download/mcs?lang=zh
```

MindCloud 的 Odin1 使用流程并没有单独固定成一个长期独立的“MindCloud 手册页面”；最实用的官方说明在：

```text
Odin1 快速启动
https://manifoldtechltd.github.io/wiki/odin_series/odin1/7.%20Odin1%20Quick%20start%20for%20ubuntu.html

Odin1 重定位地图获取手册
https://manifoldtechltd.github.io/wiki/odin_series/odin1/11.%20Relocalization%20Map%20Acquisition%20Guide.html
```

当前项目实际正式地图使用过：

```text
MindCloud v1.6.10
```

MindCloud 版本后续可以升级，但如果新版本输出异常，优先回到当前已验证版本对比。

---

## 5.5 GENISOM / 智元机器狗 SDK

```text
https://github.com/zsibot/genisom_l1_sdk
```

当前项目仓库通过：

```text
third_party/genisom_L1_sdk
```

保存该 SDK gitlink/submodule。

---

## 5.6 点云辅助工具（可选）

CloudCompare：

```text
https://www.cloudcompare.org/
```

Open3D：

```text
https://www.open3d.org/
```

这两个工具适合大 PCD 的可视化、降采样和裁剪。

---

# 6. 当前仓库结构

代码根目录：

```text
/home/robot/project/odin1_jie_nav_ws
```

主要结构：

```text
odin1_jie_nav_ws/
├── deploy_to_dog.sh
├── src/
│   ├── d1_bringup/
│   ├── d1_description/
│   ├── genisom_bridge/
│   ├── jie_3d_nav/
│   ├── jie_dog_controller/
│   └── odin_ros_driver/
└── third_party/
    └── genisom_L1_sdk/
```

各部分用途：

### `odin_ros_driver`

Odin1 ROS2 驱动，负责：

```text
USB 连接
Odin streams
odometry
high-frequency odometry
SLAM cloud
OLX recorddata
relocalization
TF
```

### `jie_3d_nav`

上游 3D 导航项目主体，主要包括：

```text
jie_map_msgs
jie_octomap
octo_planner
Web UI
```

### `jie_dog_controller`

本项目新增的真实路径跟踪控制器：

```text
/planned_path
+ Odin /odin1/odometry
+ TF
→ normalized forward / yaw command
```

发布：

```text
/dog/desired_cmd_normalized
```

### `genisom_bridge`

本项目新增的 GENISOM SDK 安全执行桥。

它是机器狗 SDK 控制权的唯一正式拥有者，负责：

```text
Web Start/Stop
SDK Role 切换
安全门
command timeout
速度限幅
joystick 映射
零命令保护
返回 remote
```

### `d1_bringup` / `d1_description`

主要为上游 `nav.launch.py` 的 package/URDF 依赖。

当前真实导航并不使用原来的 `d1_controller` 或 `d1_core` 控制机器狗。

---

# 7. 第一次拿到项目：先恢复代码环境

## 7.1 GitHub 权限

项目仓库为正式代码来源。接手人必须先获得该仓库访问权限。

PC：

```bash
mkdir -p /home/robot/project
cd /home/robot/project

git clone --recurse-submodules \
  https://github.com/yuanliang111/odin1_jie_nav_ws.git

cd /home/robot/project/odin1_jie_nav_ws
```

如果已经 clone，但 SDK 子模块未拉取：

```bash
git submodule update --init --recursive
```

检查：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
```

当前交接基线应当是：

```text
branch = main
HEAD   = f76814094ed249f12802069d53af4b8aed6a7f18
```

后续如果 main 已有新提交，以 GitHub 当前 main 为准，但要明确记录新 baseline。

---

# 8. PC 与机器狗第一次环境检查

## 8.1 PC ROS

```bash
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null && echo ROS2_OK
colcon --help >/dev/null && echo COLCON_OK
```

建议安装基础工具：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  rsync \
  git
```

如果是全新 ROS 环境，可使用：

```bash
sudo rosdep init   # 只需第一次；若提示已初始化可忽略
rosdep update
cd /home/robot/project/odin1_jie_nav_ws
rosdep install --from-paths src --ignore-src -r -y
```

项目地图处理依赖 Open3D / OctoMap / PCL；如果 `rosdep` 没完全解决，按具体 build 错误补齐，不要盲目替换版本。

---

## 8.2 机器狗 SSH

PC：

```bash
ping -c 3 192.168.0.106
ssh firefly@192.168.0.106
```

密码：

```text
firefly
```

如果 IP 改变，后续所有 `DOG_IP=`、Web URL 和 SSH 命令都必须使用真实新 IP。

---

# 9. Odin1 USB 基础检查

在机器狗：

```bash
lsusb | grep 2207:0019
```

正常应看到类似：

```text
2207:0019 Fuzhou Rockchip Electronics Company hawk
```

udev 文件：

```text
/etc/udev/rules.d/99-odin-usb.rules
```

规则：

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

如全新系统尚未配置：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-odin-usb.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

必要时重新插拔 Odin USB。

---

# 10. Odin1 三种模式必须先理解

Odin1 当前核心模式：

```text
custom_map_mode = 0   ODOM / 里程计模式
custom_map_mode = 1   SLAM / 建图模式
custom_map_mode = 2   Relocalization / 重定位模式
```

本项目正式地图生产采用的是：

```text
recorddata=1 + custom_map_mode=0
→ 采集完整 OLX
→ MindCloud 后处理
→ 导出 PCD + BIN
```

这与“直接使用 Odin SLAM mode=1 生成 BIN”是两个不同流程。

项目选择 OLX + MindCloud 的原因是：

```text
同一份原始数据
→ 同一套后处理
→ 同时得到 BIN 和 PCD
→ 更容易保持重定位地图与导航地图坐标一致
```

---

# 11. 最关键原则：BIN 和 PCD 必须来自同一份 OLX / 同一 MindCloud 结果

强烈建议：

```text
一份 OLX
    ↓
同一个 MindCloud task
    ├── BIN
    └── PCD
```

不要用 A 场景/某次扫描生成 BIN，又用 B 场景/另一次扫描生成 PCD。

否则极容易出现：

```text
Odin 认为机器人在 A 坐标系
OctoMap 在 B 坐标系
→ 规划路径与机器人实际位置整体旋转/平移错位
```

PCD 处理阶段可以：

```text
voxel downsample
crop
删除明显噪声
删除导航无关区域
```

但是不要随意对整张 PCD：

```text
整体平移
整体旋转
```

如果必须变换，要把完整 transform 明确记录并对所有地图资产统一应用。

---

# 12. 新场景地图目录规范

建议每次新建图都使用一个唯一 `scene_<timestamp>`：

```text
/home/robot/project/odin1_jie_nav_data/
└── scene_YYYYMMDD_HHMMSS/
    ├── 01_raw/
    │   └── YYYYMMDD_HHMMSS/        # Odin recorddata 整个目录
    ├── 02_mindcloud/
    │   ├── map_merged.bin
    │   └── scene_raw.pcd
    └── 03_navigation/
        ├── scene_nav.pcd
        └── octomap_package/
            ├── meta.yaml
            ├── octomap_msg.npz
            └── layers.npz
```

机器狗只需要部署运行必要资产：

```text
/home/firefly/odin1_jie_nav_data/
└── scene_.../
    ├── 02_mindcloud/
    │   └── map_merged.bin
    └── 03_navigation/
        └── octomap_package/
            ├── meta.yaml
            ├── octomap_msg.npz
            └── layers.npz
```

当前 `show_rviz=false`，因此 300MB 级 `scene_raw.pcd` 不需要为了 Web 运行强行复制到机器狗。

---

# 13. 正式地图采集：必须在机器狗上完成

## 13.1 采集前禁止运行真实导航

建图阶段只运行 Odin recorddata。

不要同时运行：

```text
real_robot_nav.launch.py
genisom_state_node
jie_dog_controller
任何其他 SDK 控制程序
```

如果需要移动机器狗采集环境，使用人工遥控/厂家正常遥控方式，确保只有一个运动控制源。

---

## 13.2 创建临时 record 配置

机器狗：

```bash
cd /home/firefly/odin1_jie_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

不要永久把正式默认 YAML 改成 `recorddata: 1`。

复制一份临时配置：

```bash
cp src/odin_ros_driver/config/control_command.yaml \
   /tmp/odin_record_odom.yaml
```

使用 Python 精确修改：

```bash
python3 - <<'PY'
import yaml
p = "/tmp/odin_record_odom.yaml"
with open(p, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

rk = cfg["register_keys"]
rk["recorddata"] = 1
rk["custom_map_mode"] = 0
rk["relocalization_map_abs_path"] = ""

with open(p, "w", encoding="utf-8") as f:
    yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
PY
```

检查：

```bash
grep -E 'recorddata|custom_map_mode|relocalization_map_abs_path' \
  /tmp/odin_record_odom.yaml
```

必须确认：

```text
recorddata: 1
custom_map_mode: 0
relocalization_map_abs_path: ''
```

---

## 13.3 启动 Odin recorddata

机器狗：

```bash
cd /home/firefly/odin1_jie_nav_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOCALHOST_ONLY=0

ros2 run odin_ros_driver host_sdk_sample \
  --ros-args \
  -p config_file:=/tmp/odin_record_odom.yaml
```

必须在日志中看到：

```text
Using config file: /tmp/odin_record_odom.yaml
Loading config file: /tmp/odin_record_odom.yaml
Loaded key: recorddata = 1
custom_map_mode = 0
Device ready and streams activated
```

如果日志仍然是 `recorddata = 0`，说明临时配置没有生效，不能继续正式扫描。

---

# 14. 如何实际扫图

以下是本项目实践建议：

1. 先让 Odin 正常开流稳定；
2. 从一个易识别位置开始；
3. 机器狗以稳定速度移动，不要猛转；
4. 尽量覆盖所有未来会导航的区域；
5. 对关键区域从不同方向重复观察；
6. 尽量形成闭环路径；
7. 避免大量人员长期挡在传感器前方；
8. 不要只扫地面，要让 Odin 看到足够多固定环境结构；
9. 扫图过程中不要重启 Odin；
10. 结束时正常 `Ctrl+C`。

当前正式基线扫描实际约：

```text
6 分钟
pose_index  ≈ 3664
cloud_index ≈ 3663
image_index ≈ 3672
```

它不是“必须 6 分钟”，只是当前场地成功案例。

---

# 15. 正确结束 OLX 录制

回到 Odin 启动终端：

```text
Ctrl+C
```

不要用：

```text
kill -9
直接断电
直接拔 Odin
```

官方说明和本项目经验都表明，异常退出可能导致 recording 中的相机标定信息不完整。

录制目录：

```text
/home/firefly/odin1_jie_nav_ws/src/odin_ros_driver/recorddata/<timestamp>/
```

例如当前正式基线：

```text
/home/firefly/odin1_jie_nav_ws/src/odin_ros_driver/recorddata/20260824_150211
```

---

# 16. OLX 完整性检查

重点检查：

```text
recorddata/<timestamp>/image/cam_in_ex.txt
```

例如：

```bash
REC=/home/firefly/odin1_jie_nav_ws/src/odin_ros_driver/recorddata/20260824_150211

ls -lah "$REC"
sed -n '1,120p' "$REC/image/cam_in_ex.txt"
```

官方手册要求 `cam_in_ex.txt` 中有完整的：

```text
Tcl_0
cam_0
image_width
image_height
相机内参/畸变参数
```

如果明显缺失，说明这次 recording 可能异常，不建议直接进入 MindCloud。

---

# 17. 把完整 OLX 从机器狗复制到 PC

这一步**必须复制整个 timestamp 目录**，不要只复制几个 cloud/pose 文件。

PC：

```bash
SCENE=scene_20260824_150211
mkdir -p /home/robot/project/odin1_jie_nav_data/$SCENE/01_raw

rsync -avP \
  firefly@192.168.0.106:/home/firefly/odin1_jie_nav_ws/src/odin_ros_driver/recorddata/20260824_150211/ \
  /home/robot/project/odin1_jie_nav_data/$SCENE/01_raw/20260824_150211/
```

复制后：

```bash
du -sh /home/robot/project/odin1_jie_nav_data/$SCENE/01_raw/20260824_150211
find /home/robot/project/odin1_jie_nav_data/$SCENE/01_raw/20260824_150211 \
  -maxdepth 2 -type f | head -50
```

---

# 18. MindCloud：必须在 Windows 环境完成后处理

官方 Odin1 重定位地图获取手册明确说明，OLX 导入 MindCloud Studio 后可同时导出：

```text
PCD 点云
BIN 重定位地图
```

## 18.1 安装 MindCloud

官方下载：

```text
https://version.manifoldtech.cn/download/mcs?lang=zh
```

如果 Ubuntu PC 没有 Windows，可以：

```text
Ubuntu PC → U盘/共享盘 → Windows PC
```

MindCloud 输出完成后，再复制回 Ubuntu 开发电脑。

---

# 19. MindCloud 导入 OLX 的详细流程

不同 MindCloud 版本 UI 字样可能略有变化，但官方流程基本一致。

1. 打开 MindCloud Studio；
2. 选择“新建任务”；
3. 点击“浏览”；
4. 选择这次 Odin OLX recording 的完整工程入口/工程文件；
5. 不要只选择某一个 cloud 文件；
6. 根据需要选择优化项；
7. 如果存在“分辨率”选项，想保留原始结果时可先使用原始分辨率；
8. 创建任务；
9. 等待 MindCloud 后处理完成；
10. 状态必须正常完成/已加载后再导出。

当前项目正式基线使用：

```text
MindCloud v1.6.10
```

---

# 20. MindCloud 导出 PCD

官方快速启动中的基本操作：

1. 在处理后的工程中选中需要导出的点云；
2. 点击保存/导出图标；
3. 文件类型选择：

```text
Point Cloud Library cloud (*.pcd)
```

4. 保存。

本项目约定命名：

```text
scene_raw.pcd
```

放回 Ubuntu PC：

```text
/home/robot/project/odin1_jie_nav_data/<scene>/02_mindcloud/scene_raw.pcd
```

当前正式基线：

```text
18,418,917 points
约 323.6 MB
```

---

# 21. MindCloud 导出 Odin 重定位 BIN

在 MindCloud 后处理结果中：

```text
后处理 / 地图相关功能
→ 导出地图
```

导出的 BIN 用于 Odin1 Relocalization。

本项目约定命名：

```text
map_merged.bin
```

放在：

```text
/home/robot/project/odin1_jie_nav_data/<scene>/02_mindcloud/map_merged.bin
```

当前正式基线 BIN：

```text
大小：1,504,154,101 bytes
SHA256：85b7ac9b529f937f8a4cbef45352ead480090de9cff6e31d7eb6ee03088ba34c
```

可验证：

```bash
sha256sum map_merged.bin
```

---

# 22. MindCloud 输出后必须形成下面结构

Ubuntu PC：

```text
02_mindcloud/
├── map_merged.bin
└── scene_raw.pcd
```

这两个文件来自同一份 OLX 和同一个 MindCloud task。

到这里才算 MindCloud 阶段 PASS。

---

# 23. 为什么不能直接拿 18M 点 PCD 做导航

原始 PCD 很大：

```text
18.4M points
323MB+
```

直接拿来做 OctoMap 会带来：

```text
PC 内存占用大
转换慢
显示卡顿
OctoMap 不必要地密
噪声被放大
```

所以必须先做导航版 PCD。

---

# 24. PCD 抽稀：在 Ubuntu 开发电脑做

当前正式基线处理：

```text
scene_raw.pcd
18.4M points
    ↓
0.1 m voxel downsample
    ↓
手工裁掉场外明显异常
    ↓
scene_nav.pcd
70,040 points
约 1.1 MB
```

导航 PCD：

```text
/home/robot/project/odin1_jie_nav_data/scene_20260824_150211/03_navigation/scene_nav.pcd
```

字段：

```text
x y z rgb
DATA binary
```

---

# 25. 一个可复现的 Open3D 抽稀脚本

在 Ubuntu PC 创建：

```text
/home/robot/project/downsample_pcd.py
```

内容：

```python
#!/usr/bin/env python3
import open3d as o3d

INPUT_PCD = "/home/robot/project/odin1_jie_nav_data/scene_xxx/02_mindcloud/scene_raw.pcd"
OUTPUT_PCD = "/home/robot/project/odin1_jie_nav_data/scene_xxx/03_navigation/scene_ds.pcd"
VOXEL_SIZE = 0.10

print("Loading PCD...")
pcd = o3d.io.read_point_cloud(INPUT_PCD)
print("Original points:", len(pcd.points))

pcd_ds = pcd.voxel_down_sample(VOXEL_SIZE)
print("Downsampled points:", len(pcd_ds.points))

ok = o3d.io.write_point_cloud(
    OUTPUT_PCD,
    pcd_ds,
    write_ascii=False,
    compressed=True,
)
print("Saved:", OUTPUT_PCD, ok)
```

运行：

```bash
python3 /home/robot/project/downsample_pcd.py
```

如果 Open3D 未安装：

```bash
python3 -c 'import open3d; print(open3d.__version__)'
```

再根据当前 Python/Ubuntu 环境安装对应 Open3D。

---

# 26. PCD 手工裁剪原则

抽稀后，使用 CloudCompare 或项目自带 PCD GUI 进行可视化裁剪。

可以删除：

```text
明显场外漂点
完全不属于导航区域的远处物体
扫描时的孤立噪声
非常明显的异常点簇
```

不要删除：

```text
真实墙体
台阶边缘
柱子
长期固定障碍物
地面结构
```

不要把整张地图手工旋转/平移。

最后保存为：

```text
scene_nav.pcd
```

---

# 27. PCD → OctoMap：必须在 PC 完成

PC：

```bash
cd /home/robot/project/odin1_jie_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果尚未 build：

```bash
colcon build --packages-select \
  jie_map_msgs \
  jie_octomap \
  octo_planner

source install/setup.bash
```

启动 PCD 导入：

```bash
ros2 launch jie_octomap import_pcd_map.launch.py \
  resolution:=0.2 \
  voxel_downsample_m:=0.0 \
  min_points_per_voxel:=1 \
  min_cluster_voxels:=1
```

当前正式基线就是这些参数。

---

# 28. 为什么 `voxel_downsample_m:=0.0`

因为 PCD 已经提前做过：

```text
0.1 m downsample
```

因此 OctoMap converter 内不再做第二次下采样：

```text
voxel_downsample_m=0.0
```

项目 `import_pcd_map.launch.py` 也明确注明：如果 GUI 已经输出预处理临时 PCD，就避免 double downsampling。

---

# 29. PCD 导入 GUI 怎么用

启动后会出现 `PCD 地图导入` GUI。

关键输入：

```text
PCD 文件：scene_nav.pcd
Octomap 分辨率：0.2
预处理降采样：如果 scene_nav 已抽稀，使用 0 或不要再次强降采样
每体素最少点数：1
最小连通体素数：1
```

GUI 支持：

```text
PCD 读取
预览
voxel downsample
统计离群点滤波
选择/删除
OctoMap 转换
查看 occupied 层
查看 preblocked 层
查看 traversable 层
查看 risk cost 层
选择 start/goal
显示规划 path
保存 Navigation Map Package
```

---

# 30. 当前 PCD→OctoMap 成功基线

当前 `scene_nav.pcd`：

```text
source_points   = 70040
counted_voxels  = 14942
kept_voxels     = 14942
occupied_voxels = 15890
```

OctoMap：

```text
resolution = 0.2 m
frame_id   = map
```

这组数值是当前地图的参考，不要求新地图一模一样。

---

# 31. “进一步加工 OctoMap”具体是什么意思

本项目不是只把 PCD 塞进 OctoMap 就结束。

`jie_path_node` 会基于 OctoMap 生成导航相关层：

```text
occupied
preblocked
traversable
risk cost
```

当前关键 planner 参数：

```yaml
robot_radius: 0.25
snap_search_radius_cells: 12
require_ground_support: true
strict_direct_ground_support: false
ground_support_xy_radius_cells: 1
ground_support_depth_cells: 1
enable_preblocked_costmap: true
preblocked_costmap_radius_cells: 3
preblocked_costmap_weight: 2.5
```

理解方式：

```text
OctoMap = 原始 3D 占据关系

planner layers = 为“机器狗到底能不能走、哪里风险高、哪里要提前阻塞”进行的导航加工
```

当前基线：

```text
preblocked_count  = 8069
traversable_count = 2399
risk_cost_count   = 1964
```

---

# 32. 转完 OctoMap 后先做一次离线路径规划

在 GUI 中选一个明显可走的 Start 和 Goal。

当前历史成功案例：

```text
Start = [0.300, -0.100, 0.100]
Goal  = [7.300,  0.100, -0.100]
```

输出：

```text
A* path found in 507 iterations
waypoints = 36
```

ROS topic：

```text
/planned_path
类型：nav_msgs/msg/Path
frame_id：map
```

新地图至少要先在 PC 上确认能规划出一条合理路径，再部署到真机。

---

# 33. 保存 Navigation Map Package

建议优先使用 GUI 的保存功能。

地图包目录：

```text
/home/robot/project/odin1_jie_nav_data/<scene>/03_navigation/octomap_package
```

如果需要命令行保存，可调用：

```bash
ros2 service call \
  /map_package_manager/save_package \
  jie_map_msgs/srv/SaveNavigationMapPackage \
  "{package_path: '/home/robot/project/odin1_jie_nav_data/<scene>/03_navigation/octomap_package', overwrite: true}"
```

成功后必须有：

```text
octomap_package/
├── meta.yaml
├── octomap_msg.npz
└── layers.npz
```

---

# 34. 三个地图包文件分别是什么

## `octomap_msg.npz`

保存序列化后的 OctoMap message：

```text
binary
OctoMap id
resolution
frame_id
data
```

## `layers.npz`

保存 planner 导航层：

```text
preblocked_points
traversable_points
risk_points
risk_intensity
```

## `meta.yaml`

保存地图和规划参数元数据，例如：

```yaml
map_id: imported_pcd_map
frame_id: map
resolution: 0.2
```

以及 planner 参数和 layer 数量。

---

# 35. 当前正式 Map Package 基线

当前 PC：

```text
/home/robot/project/odin1_jie_nav_data/scene_20260824_150211/03_navigation/octomap_package
```

当前 meta：

```yaml
map_id: imported_pcd_map
frame_id: map
resolution: 0.2

planner:
  robot_radius: 0.25
  snap_search_radius_cells: 12
  require_ground_support: true
  strict_direct_ground_support: false
  ground_support_xy_radius_cells: 1
  ground_support_depth_cells: 1
  enable_preblocked_costmap: true
  preblocked_costmap_radius_cells: 3
  preblocked_costmap_weight: 2.5

layers:
  preblocked_count: 8069
  traversable_count: 2399
  risk_cost_count: 1964
```

当前 SHA256：

```text
meta.yaml
f1420db09222c32f7a0814ea8280e3bddeef9318d19f33b4425a99651f27e083

octomap_msg.npz
49b4020f618278ad7b1622c8fbf662cf2fac6640c2b8d1da32f15598521ff326

layers.npz
a79d6aa5be9e69c01c8722406b713d9433644087e110931d2636751331ba68dd
```

---

# 36. 把地图资产从 PC 部署回机器狗

## 36.1 创建机器狗目录

PC：

```bash
ssh firefly@192.168.0.106 \
  'mkdir -p /home/firefly/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud \
           /home/firefly/odin1_jie_nav_data/scene_20260824_150211/03_navigation/octomap_package'
```

## 36.2 部署 BIN

```bash
rsync -avP \
  /home/robot/project/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/map_merged.bin \
  firefly@192.168.0.106:/home/firefly/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/
```

## 36.3 部署 Navigation Map Package

```bash
rsync -avP \
  /home/robot/project/odin1_jie_nav_data/scene_20260824_150211/03_navigation/octomap_package/ \
  firefly@192.168.0.106:/home/firefly/odin1_jie_nav_data/scene_20260824_150211/03_navigation/octomap_package/
```

---

# 37. 部署后必须做 SHA 校验

PC：

```bash
sha256sum \
  /home/robot/project/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/map_merged.bin
```

机器狗：

```bash
sha256sum \
  /home/firefly/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/map_merged.bin
```

两个 SHA 必须一致。

地图包三个文件同理。

---

# 38. 新地图需要改哪些源码配置

PC 是真值来源，**不要直接在机器狗上手改**。

主要修改：

```text
src/jie_3d_nav/octo_planner/config/nav_params.yaml
```

三个关键路径：

```yaml
relocalization_bin_file: /home/firefly/odin1_jie_nav_data/<scene>/02_mindcloud/map_merged.bin
relocalization_pcd_file: /home/firefly/odin1_jie_nav_data/<scene>/02_mindcloud/scene_raw.pcd
map_package_dir: /home/firefly/odin1_jie_nav_data/<scene>/03_navigation/octomap_package
```

以及：

```text
src/jie_3d_nav/jie_octomap/config/loc_control_command.yaml
```

关键：

```yaml
recorddata: 0
custom_map_mode: 2
relocalization_map_abs_path: "/home/firefly/odin1_jie_nav_data/<scene>/02_mindcloud/map_merged.bin"
```

注意：正式导航 runtime 中 `nav.launch.py` 会读取配置并同步 Odin 的重定位 BIN 路径。

---

# 39. 不要直接运行上游 `nav.launch.py` 做真实机器狗运动

当前真实机器人正式入口不是：

```bash
ros2 launch octo_planner nav.launch.py
```

因为上游默认会有：

```text
launch_d1_core=true
launch_controller=true
use_static_odom_to_base=true（来自默认配置）
launch_rosbridge=true
```

这不是当前 GENISOM 真机安全架构。

本项目新增了：

```text
real_robot_nav.launch.py
```

它明确覆盖：

```text
launch_rviz=false
launch_map_gui=false
launch_d1_core=false
launch_controller=false
publish_d1_odom=false
use_static_odom_to_base=false
launch_planner=true
launch_web=true
launch_rosbridge=false
```

---

# 40. 为什么 rosbridge 单独启动

项目中已经实测过“nav launch 内集成 rosbridge”不够稳定。

因此当前正式 baseline：

```text
终端1：real_robot_nav.launch.py
终端2：独立 rosbridge websocket
终端3：只读检查 / emergency stop
```

不要为了“少一个终端”再把 rosbridge 强行塞回主 launch，除非重新做稳定性验证。

---

# 41. PC 源码修改后的标准 Git 流程

每次修改前：

```bash
cd /home/robot/project/odin1_jie_nav_ws

git branch --show-current
git rev-parse HEAD
git status --short
```

当前只有：

```text
main
```

修改后：

```bash
git diff --check
colcon build --packages-select <相关包>
```

再由负责人：

```bash
git add <明确文件>
git commit -m "..."
git push origin main
```

不要 `git add .` 盲目把临时文件加进去。

---

# 42. 从 PC 差分部署代码到机器狗

仓库自带：

```text
deploy_to_dog.sh
```

非常重要：脚本内部默认 `DOG_IP` 可能是历史值，所以交接时**必须显式传真实 IP**。

例如当前：

```bash
cd /home/robot/project/odin1_jie_nav_ws

DOG_IP=192.168.0.106 \
DRY_RUN=0 \
BUILD=1 \
DELETE=0 \
ALLOW_DIRTY=0 \
BUILD_PACKAGES="jie_map_msgs jie_octomap odin_ros_driver octo_planner jie_dog_controller genisom_bridge" \
./deploy_to_dog.sh
```

如果是全新机器狗 workspace，建议先把依赖和所有必要包 build 完整；之后日常修改只 build 相关包即可。

成功必须看到：

```text
deployment result: SUCCESS
```

脚本会主动排除：

```text
build/
install/
log/
odin_ros_driver/recorddata/
odin_ros_driver/map/
```

所以地图数据需要按前面的 rsync 单独管理。

---

# 43. 机器狗第一次完整编译建议

在机器狗：

```bash
cd /home/firefly/odin1_jie_nav_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  d1_description \
  d1_bringup \
  jie_map_msgs \
  jie_octomap \
  odin_ros_driver \
  octo_planner \
  jie_dog_controller \
  genisom_bridge
```

然后：

```bash
source install/setup.bash
```

还需要确保：

```text
rosbridge_server
rmw_zenoh_cpp
```

在机器狗环境可见。

检查：

```bash
ros2 pkg list | grep '^rosbridge_server$'
ros2 pkg list | grep '^rmw_zenoh_cpp$' || true
```

---

# 44. Odin1 重定位正式配置

正式导航重定位：

```text
custom_map_mode = 2
recorddata       = 0
relocalization_map_abs_path = map_merged.bin
```

启动后关键成功日志：

```text
Relocalization map set successfully
Device ready and streams activated
relocalization success!
```

如果没有 `relocalization success!`，不要开始真实导航。

官方也指出：Odin 可能需要移动一段距离才能重定位，不要只在完全静止状态立刻判失败。

---

# 45. Odin1 当前正式 ROS 数据

当前已验证：

```text
/odin1/odometry
/odin1/odometry_highfreq
/odin1/cloud_slam
```

Odometry：

```text
frame_id       = odom
child_frame_id = imu
```

当前 controller 正式使用：

```text
/odin1/odometry
```

highfreq 仍可用于诊断。

---

# 46. 一个非常容易踩坑的 TF：Odin 发布的是 `odom → map`

当前 Odin driver 重定位 TF 的实际 authority：

```text
parent = odom
child  = map
```

即：

```text
odom → map
```

这和很多 ROS 导航教程常见的：

```text
map → odom
```

方向相反。

所以：

1. 不要因为“看起来反了”就自己再发布一个 map→odom；
2. 不要制造双 TF authority；
3. `tf2_echo map odom` 会自动显示逆变换，这是正常的；
4. `jie_dog_controller` 已通过 tf2 正确将 Odin odom pose 转到 path 的 `map` frame。

---

# 47. 当前机器狗控制链

正式运动链：

```text
/planned_path
        ↓
jie_dog_controller
        ↓
/dog/desired_cmd_normalized
        ↓
genisom_state_node
        ↓
GENISOM SDK
        ↓
机器狗
```

`jie_dog_controller` 负责：

```text
lookahead tracking
Odin pose → map
heading error
turn-in-place hysteresis
forward gating
goal reached
```

`genisom_state_node` 负责：

```text
SDK connection
control mode check
actuation enable/disable
command watchdog
forward/yaw clamp
joystick mapping
zero command
return remote
```

---

# 48. GENISOM joystick 映射：不要按官方文档想当然

本项目真机实测发现，该机器狗当前 firmware 的实际轴映射是：

```text
axis 0 = 前进/后退
axis 1 = 横移
axis 2 = yaw 原地转向
```

因此当前正式配置：

```yaml
forward_joystick_index: 0
yaw_joystick_index: 2
yaw_joystick_sign: 1.0
```

**axis 1 在当前导航中必须保持 0。**

之前错误把 yaw 发到 axis 1 时，机器狗实际表现为横移，这个问题已经定位并修复。

---

# 49. 当前安全 runtime 参数

文件：

```text
src/genisom_bridge/config/real_nav_runtime.yaml
```

当前默认：

```yaml
actuation_capable: false
max_forward_normalized: 0.05
max_yaw_normalized: 0.50
command_timeout_sec: 0.30
forward_joystick_index: 0
yaw_joystick_index: 2
yaw_joystick_sign: 1.0

lookahead_distance: 0.40
turn_in_place_enter_angle: 0.60
turn_in_place_exit_angle: 0.30
```

注意：

```text
max_forward_normalized
```

是 SDK joystick normalized 值，**不是 m/s**。

当前最稳妥真实 baseline 是：

```text
0.05
```

---

# 50. 完整冷启动：终端 1

PC 开 SSH：

```bash
ssh firefly@192.168.0.106
```

机器狗终端 1：

```bash
cd /home/firefly/odin1_jie_nav_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOCALHOST_ONLY=0

ros2 launch octo_planner real_robot_nav.launch.py \
  actuation_capable:=true \
  max_forward_normalized:=0.05 \
  max_yaw_normalized:=0.50 \
  web_http_port:=8088
```

这一个 launch 会启动主要系统：

```text
Odin driver
OctoMap/map package manager
jie_path_node
Web HTTP server
genisom_state_node
jie_dog_controller
```

它不会启动：

```text
d1_controller
d1_core
static odom->base_link
rosbridge
```

---

# 51. 终端 1 启动成功的判据

至少要看到：

```text
Relocalization map set successfully
Device ready and streams activated
relocalization success!
```

地图：

```text
autoloaded map package: .../octomap_package
```

planner：

```text
jie_path_node started
```

Web：

```text
Serving HTTP no-cache on 0.0.0.0 port 8088
```

---

# 52. 已知 Odin 日志警告如何判断

当前真机曾出现：

```text
Error: Missing camera node 'cam_0'
Failed to initialize point cloud renderer
读取YAML文件失败: bad conversion
Failed to set SCHED_FIFO
cross-stream gap ... -> reset
```

这些在当前导航配置中曾经与下面成功状态同时出现：

```text
Device ready and streams activated
relocalization success!
/odin1/odometry 正常
/odin1/odometry_highfreq 正常
```

因此不能只看到一行 ERROR 就判断整个导航失败。

真正要看：

```text
USB 是否连接
BIN 是否 set success
streams 是否 activated
relocalization 是否 success
odometry 是否持续发布
```

如果这些失败，再 STOP。

---

# 53. 完整冷启动：终端 2 独立 rosbridge

新 SSH：

```bash
ssh firefly@192.168.0.106
```

执行：

```bash
cd /home/firefly/odin1_jie_nav_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOCALHOST_ONLY=0

ros2 run rosbridge_server rosbridge_websocket --ros-args \
  -p port:=9090 \
  -p address:="0.0.0.0"
```

保持运行。

---

# 54. 完整冷启动：终端 3 安全检查

```bash
ssh firefly@192.168.0.106
```

执行：

```bash
cd /home/firefly/odin1_jie_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOCALHOST_ONLY=0

printf '\n===== CORE NODES =====\n'
echo "HOST_SDK_COUNT=$(ros2 node list | grep -c '^/host_sdk_sample$')"
echo "PLANNER_COUNT=$(ros2 node list | grep -c '^/jie_path_node$')"
echo "GENISOM_COUNT=$(ros2 node list | grep -c '^/genisom_state_node$')"
echo "CONTROLLER_COUNT=$(ros2 node list | grep -c '^/jie_dog_controller$')"
echo "ROSBRIDGE_COUNT=$(ros2 node list | grep -c '^/rosbridge_websocket$')"

printf '\n===== FORBIDDEN NODES =====\n'
echo "D1_CONTROLLER_COUNT=$(ros2 node list | grep -c '^/d1_controller$')"
echo "D1_CORE_COUNT=$(ros2 node list | grep -c '^/d1_core_node$')"
echo "STATIC_ODOM_BASE_COUNT=$(ros2 node list | grep -c '^/static_odom_to_base_link$')"

printf '\n===== ODIN =====\n'
if timeout 5s ros2 topic echo /odin1/odometry_highfreq --once >/dev/null 2>&1; then
  echo ODIN_HIGHFREQ_STREAM=PASS
else
  echo ODIN_HIGHFREQ_STREAM=FAIL
fi

printf '\n===== SPEED =====\n'
ros2 param get /genisom_state_node max_forward_normalized
ros2 param get /genisom_state_node max_yaw_normalized
ros2 param get /genisom_state_node forward_joystick_index
ros2 param get /genisom_state_node yaw_joystick_index
ros2 param get /genisom_state_node yaw_joystick_sign

printf '\n===== ACTUATION =====\n'
timeout 5s ros2 topic echo /dog/actuation_debug --once --full-length

printf '\n===== CONTROLLER =====\n'
timeout 5s ros2 topic echo /dog/control_debug --once --full-length
```

---

# 55. 真实运动前 READY 条件

必须：

```text
HOST_SDK_COUNT=1
PLANNER_COUNT=1
GENISOM_COUNT=1
CONTROLLER_COUNT=1
ROSBRIDGE_COUNT=1
```

必须：

```text
D1_CONTROLLER_COUNT=0
D1_CORE_COUNT=0
STATIC_ODOM_BASE_COUNT=0
```

必须：

```text
ODIN_HIGHFREQ_STREAM=PASS
```

必须：

```text
forward_joystick_index=0
yaw_joystick_index=2
yaw_joystick_sign=1.0
```

还没点 Web Start 时必须：

```text
state=DISABLED
actuation_enabled=false
sdk_control_confirmed=false
joystick_0=0
joystick_1=0
joystick_2=0
joystick_3=0
```

全部满足才是：

```text
READY
```

---

# 56. Web 页面

电脑浏览器：

```text
http://192.168.0.106:8088
```

如果 IP 改变，把 URL 中 IP 改成真实机器狗 IP。

rosbridge websocket：

```text
ws://192.168.0.106:9090
```

Web 前端已配置使用 rosbridge 与 ROS2 通信。

---

# 57. Web 正常导航操作顺序

推荐第一次：

1. 打开 Web；
2. 确认地图显示；
3. 确认机器人位置在合理位置；
4. 选择一个短距离、无遮挡、平面目标；
5. 等待紫色规划路径出现；
6. 先看 `/dog/control_debug`；
7. bridge 仍应是 `DISABLED`；
8. 点击 Web“开始导航”；
9. 机器狗先按需要原地转向；
10. 对准后前进；
11. 到目标后 controller 输出 0；
12. 点击 Web“停止导航”；
13. 确认 bridge 回到 `DISABLED`。

---

# 58. Controller 关键状态

无路径：

```text
NO_PATH
```

路径存在但无有效 target：

```text
NO_VALID_TARGET
```

正在跟踪：

```text
TRACKING_VIRTUAL
```

到达：

```text
GOAL_REACHED_VIRTUAL
```

当前已真实验证到达案例：

```text
distance ≈ 0.054 m
state = GOAL_REACHED_VIRTUAL
```

---

# 59. turn-in-place 的意义

如果目标方向与机头夹角较大：

```text
turn_in_place=true
forward=0
yaw!=0
```

先原地转。

当夹角降到退出阈值附近：

```text
turn_in_place=false
```

才开始前进。

当前：

```text
enter = 0.60 rad
exit  = 0.30 rad
```

这个设计是为了解决“目标在侧后方时机器狗边走边摆、路线不自然”的问题。

---

# 60. Web Start / Stop 是双层安全门

启动主 launch 时：

```text
actuation_capable:=true
```

只表示：

```text
该进程允许未来申请执行能力
```

并不会自动让机器狗动。

真正进入运动还需要：

```text
Web /start_navigation
```

之后：

```text
bridge ACTIVE
SDK control confirmed
非零命令才可能发送
```

Web Stop：

```text
/stop_navigation
→ zeros
→ disable
→ SDK remote
→ DISABLED
```

---

# 61. 紧急停止

首选：

```text
Web → 停止导航
```

如果 Web 不方便，机器狗终端 3：

```bash
ros2 service call \
  /dog/set_actuation_enabled \
  std_srvs/srv/SetBool \
  "{data: false}"
```

之后确认：

```bash
timeout 5s ros2 topic echo /dog/actuation_debug --once --full-length
```

应看到：

```text
state=DISABLED
actuation_enabled=false
sdk_control_confirmed=false
joystick_0=0
joystick_1=0
joystick_2=0
joystick_3=0
```

---

# 62. 修改机器狗速度

当前主 launch 已暴露：

```text
max_forward_normalized
max_yaw_normalized
```

例如：

```bash
ros2 launch octo_planner real_robot_nav.launch.py \
  actuation_capable:=true \
  max_forward_normalized:=0.05 \
  max_yaw_normalized:=0.50 \
  web_http_port:=8088
```

要改变前进速度上限，重启主 launch 并改：

```text
max_forward_normalized:=0.08
```

或其他值。

注意：它不是 m/s。

推荐从：

```text
0.05
```

逐步提高，不要第一次直接大幅增加。

---

# 63. 重要调试 topic

Odin：

```text
/odin1/odometry
/odin1/odometry_highfreq
```

地图：

```text
/octomap
/octomap_occupied_markers
/preblocked_cells_markers
/traversable_cells_markers
/risk_cost_cells
```

规划：

```text
/start_point
/goal_point
/planned_path
```

Controller：

```text
/dog/control_debug
/dog/desired_cmd_normalized
```

Bridge：

```text
/dog/state
/dog/actuation_debug
```

Web motion gate：

```text
/start_navigation
/stop_navigation
```

---

# 64. 常用只读诊断命令

```bash
ros2 node list
ros2 topic list
ros2 topic info /dog/desired_cmd_normalized -v
ros2 topic echo /dog/control_debug --once --full-length
ros2 topic echo /dog/actuation_debug --once --full-length
ros2 topic echo /odin1/odometry --once --full-length
ros2 topic echo /odin1/odometry_highfreq --once --full-length
ros2 param get /genisom_state_node max_forward_normalized
ros2 param get /genisom_state_node yaw_joystick_index
```

---

# 65. 绝对禁止同时存在多个运动控制源

当前正式设计：

```text
jie_dog_controller
    ↓
/dog/desired_cmd_normalized
    ↓
genisom_state_node
    ↓
GENISOM SDK
```

不要同时运行：

```text
上游 d1_controller
另一个 GENISOM SDK demo
手工 joystick SDK 程序
另一个自写 velocity publisher
udp_dog_move.sh
```

任何时候：

```text
一个真实机器狗 = 一个 SDK 执行 owner
```

---

# 66. `udp_dog_move.sh`

历史 PC 工作区曾存在：

```text
udp_dog_move.sh
```

它没有进入 GitHub main。

不要因为看到本地旧文件就执行它。

正式导航与它无关。

---

# 67. 当前 Web/真实导航已验证的典型最终状态

到达目标并 Stop 后：

Bridge：

```text
state="DISABLED"
actuation_enabled=false
control_source="none"
last_enable_source="web"
sdk_control_confirmed=false
joystick_0=0
joystick_1=0
joystick_2=0
joystick_3=0
```

Controller：

```text
state="GOAL_REACHED_VIRTUAL"
desired_forward_cmd=0
desired_yaw_cmd=0
```

---

# 68. 当前正式地图资产

正式场景：

```text
scene_20260824_150211
```

机器狗原始录制：

```text
/home/firefly/odin1_jie_nav_ws/src/odin_ros_driver/recorddata/20260824_150211
```

PC raw：

```text
/home/robot/project/odin1_jie_nav_data/scene_20260824_150211/01_raw/20260824_150211
```

PC MindCloud：

```text
/home/robot/project/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/map_merged.bin
/home/robot/project/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/scene_raw.pcd
```

PC navigation：

```text
/home/robot/project/odin1_jie_nav_data/scene_20260824_150211/03_navigation/scene_nav.pcd
/home/robot/project/odin1_jie_nav_data/scene_20260824_150211/03_navigation/octomap_package
```

机器狗 runtime：

```text
/home/firefly/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/map_merged.bin
/home/firefly/odin1_jie_nav_data/scene_20260824_150211/03_navigation/octomap_package
```

---

# 69. 当前 Git 基线

仓库：

```text
https://github.com/yuanliang111/odin1_jie_nav_ws.git
```

只保留：

```text
main
```

当前交接 baseline：

```text
f76814094ed249f12802069d53af4b8aed6a7f18
feat: expose real navigation speed limits
```

以前用于开发的 feature/fix 分支已经清理，不需要恢复。

---

# 70. 新场景从 0 到可导航的完整清单

以后如果换场地，按这个顺序，不要跳步：

```text
[1] 机器狗 Odin USB PASS
[2] 创建临时 recorddata=1 / mode=0 配置
[3] 机器狗实地 OLX 录制
[4] 正常 Ctrl+C
[5] 检查 cam_in_ex.txt
[6] rsync 整个 recording 到 PC
[7] Windows MindCloud 新建 task
[8] MindCloud 后处理完成
[9] 导出 map_merged.bin
[10] 导出 scene_raw.pcd
[11] BIN/PCD 放回 Ubuntu PC 同一 scene 目录
[12] scene_raw.pcd 0.1m 级抽稀
[13] 手工裁掉明显异常，不平移/旋转整图
[14] 生成 scene_nav.pcd
[15] PC 启动 import_pcd_map.launch.py
[16] resolution=0.2
[17] PCD→OctoMap
[18] 检查 occupied/preblocked/traversable/risk layers
[19] PC 上先做一条 A* 路径
[20] 保存 octomap_package
[21] 检查 meta.yaml + 两个 npz
[22] rsync BIN 到机器狗
[23] rsync map package 到机器狗
[24] SHA256 校验
[25] PC 修改 nav_params.yaml / loc_control_command.yaml
[26] build
[27] commit / push main
[28] deploy_to_dog.sh
[29] 机器狗主 launch
[30] 等 relocalization success
[31] 独立 rosbridge
[32] safety precheck
[33] Web 短路径
[34] bridge DISABLED 时才允许点 Start
[35] 真机短距离验证
[36] Web Stop
[37] 最终 bridge DISABLED
```

---

# 71. PASS / STOP 判据

## 地图采集 PASS

```text
recorddata=1 确认生效
完整 recording 目录生成
cam_in_ex.txt 完整
正常 Ctrl+C
```

## MindCloud PASS

```text
task 处理成功
BIN 导出成功
PCD 导出成功
BIN/PCD 来自同一次 task
```

## OctoMap PASS

```text
scene_nav.pcd 可读
OctoMap frame=map
resolution 正确
occupied 非空
planner layers 非空
能规划路径
map package 三文件保存成功
```

## Relocalization PASS

```text
Relocalization map set successfully
Device ready and streams activated
relocalization success!
/odin1/odometry 有持续数据
```

## 真机导航 READY

```text
所有核心节点唯一
禁止节点为 0
rosbridge 唯一
Odin stream PASS
bridge DISABLED
四轴 joystick=0
规划路径合理
```

## STOP

出现任一：

```text
Odin 未重定位
Odometry 大跳
两个 genisom_state_node
两个 jie_dog_controller
d1_controller 被启动
另一个 SDK 程序占控制权
joystick_1 非 0
Web 未 Start 就 ACTIVE
机器狗明显横移
持续原地错误旋转
路径与地图明显错位
地图/机器人坐标整体偏移
```

立即停止。

---

# 72. 已经做过、不要重复浪费时间的事情

当前项目已经确认：

```text
Odin ARM64 build
USB udev
recorddata 配置 override
OLX 正式录制
MindCloud BIN + PCD
0.1m PCD 抽稀
OctoMap 0.2m
3D A*
Navigation Map Package
BIN 真机重定位
Odin TF/odometry
Web 显示
rosbridge
GENISOM SDK bridge
joystick 真实轴映射
turn-in-place
Web Start/Stop
真实短距离自主到达
启动参数化速度
```

不要因为换了接手人就从“怀疑一切”开始重做。

先复用当前 baseline，只有真实输出出现新问题时才回退排查。

---

# 73. 已经冻结的探索方向

此前尝试过：

```text
image mask 去除 RAW DTOF 机身点
OLX + Pose 离线 self-filter
```

没有形成足够可靠收益，当前主线已经冻结。

不要在没有明确地图质量阻塞的情况下重新投入时间。

---

# 74. 后续真正值得做的事情

平面导航当前已经跑通。

后续优先级可以是：

```text
1. 把当前平面 baseline 稳定运行若干场次
2. 做速度实测标定：normalized joystick → 实际 m/s / rad/s
3. 做更长路径和多拐点回归
4. 检查长期运行 rosbridge/Odin 稳定性
5. 做不同新场景地图复现
6. 之后再进入坡道/高差真实 3D 跟踪
7. 最后再考虑进一步减少启动终端、systemd、自启动等工程化
```

当前不要优先破坏已经可用的平面链路。

---

# 75. 给接手人的最终一句话

如果只记住一件事，请记住：

> **这个项目不是“把 Odin 点云显示出来”就结束，而是要把同一次 Odin 扫描的数据分别加工成“重定位 BIN”和“导航 OctoMap”，再用 Odin 实时定位去驱动 jie_3d_nav 的路径跟踪，最后通过唯一的 GENISOM SDK bridge 安全控制真实机器狗。地图采集在机器狗，地图后处理和代码修改在 PC，处理后的 BIN/Map Package 再部署回机器狗。真实运动前永远先检查定位、TF、路径、唯一控制源和 bridge DISABLED 状态。**

---

# 附录 A：当前最快启动命令（机器狗）

## 终端 1

```bash
cd /home/firefly/odin1_jie_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOCALHOST_ONLY=0

ros2 launch octo_planner real_robot_nav.launch.py \
  actuation_capable:=true \
  max_forward_normalized:=0.05 \
  max_yaw_normalized:=0.50 \
  web_http_port:=8088
```

## 终端 2

```bash
cd /home/firefly/odin1_jie_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOCALHOST_ONLY=0

ros2 run rosbridge_server rosbridge_websocket --ros-args \
  -p port:=9090 \
  -p address:="0.0.0.0"
```

## 浏览器

```text
http://192.168.0.106:8088
```

---

# 附录 B：当前最快急停命令

```bash
ros2 service call \
  /dog/set_actuation_enabled \
  std_srvs/srv/SetBool \
  "{data: false}"
```

---

# 附录 C：当前正式地图路径

```text
BIN:
/home/firefly/odin1_jie_nav_data/scene_20260824_150211/02_mindcloud/map_merged.bin

Map Package:
/home/firefly/odin1_jie_nav_data/scene_20260824_150211/03_navigation/octomap_package
```

---

# 附录 D：交接后第一天建议做的验证

不要第一天就重新建图。

建议先：

```text
1. clone GitHub main
2. 检查 commit
3. SSH 机器狗
4. 检查 Odin USB
5. 启动当前正式 main launch
6. 等 relocalization success
7. 启动 rosbridge
8. 打开 Web
9. 只读检查节点、odometry、bridge
10. 选择 0.5~1m 短目标
11. 确认 path
12. Start
13. 到达
14. Stop
15. bridge DISABLED
```

如果这条当前 baseline 能跑通，再开始任何新地图、新速度或 3D 坡道工作。

