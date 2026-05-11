# WBC_Deploy 控制器

基于强化学习和动作跟踪的人形机器人全身控制部署系统。

[English](README.md) | 中文

## 功能特性

- **状态机控制**：包含 Passive（阻尼保护）、Loco（行走）和 WBC（全身控制）等多种 FSM 状态
- **动作跟踪**：实时跟踪重定向到 Unitree G1 人形机器人的 LAFAN1 动作数据集
- **ONNX Runtime**：使用 ONNX 模型进行快速推理
- **可配置**：基于 JSON 的配置系统，便于模式切换和参数调整

## 环境要求

- CMake >= 3.14
- C++17 编译器
- CUDA
- 依赖库：
  - unitree_sdk2
  - **ONNX Runtime 1.22.0**（见下方安装说明）
  - Eigen3
  - nlohmann_json >= 3.7.3
  - Boost

### 安装 ONNX Runtime

下载并解压 ONNX Runtime 1.22.0 到 `controller/` 目录：

**x64 平台（仿真）：**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

**aarch64 平台（真实机器人）：**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

## 编译

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

### 可选：开启千二科技 license 验证

默认编译不会强制校验 license，便于开发调试：

```bash
cmake ..
```

如需在程序启动前强制校验本地授权文件，编译时打开宏：

```bash
cmake .. -DENABLE_QIANER_LICENSE_AUTH=ON
make -j4
```

关闭该功能：

```bash
cmake .. -DENABLE_QIANER_LICENSE_AUTH=OFF
make -j4
```

开启后，`wbc_fsm` 启动时会先做本地离线校验：

- 使用 `ZJUDES.crt` 验证 `.lic` 文件中的 RSA 签名。
- 检查 license payload 中的 MAC 是否等于当前机器人网卡 MAC。
- 检查 license 是否过期。
- 校验失败时直接退出，不启动机器人控制逻辑。

默认配置文件位于：

```text
config/qianer_auth.json
```

当前工程启动时会先读取该配置文件，再读取环境变量覆盖值。这样 Ubuntu 笔记本测试和真实机器人部署不需要改代码，只需要改配置文件里的网卡名和路径。

配置示例：

```text
{
  "cert_path": "../qianer_auth_project/keys/ZJUDES.crt",
  "license_path": "license/qianer_license.lic",
  "iface": "wlp3s0"
}
```

字段说明：
- `cert_path`：验证 license 签名用的证书路径。相对路径会按 `unitree_g1` 工程根目录解析。
- `license_path`：本地 license 文件路径。相对路径会按 `unitree_g1` 工程根目录解析。
- `iface`：用于读取 MAC 地址并和 license 绑定 MAC 对比的网卡名。Ubuntu 笔记本可用 `wlp3s0`，真实机器人通常用 `eth0`，不要使用 `lo`。

也可以临时用环境变量覆盖配置文件，优先级高于 `config/qianer_auth.json`：

```bash
export QIANER_AUTH_CERT_PATH=/opt/qianer-auth/keys/ZJUDES.crt
export QIANER_AUTH_LICENSE_PATH=/home/unitree/unitree_g1/license/qianer_license.lic
export QIANER_AUTH_IFACE=wlp3s0
```

如果开启宏，需要 Ubuntu 安装额外依赖：

```bash
sudo apt update
sudo apt install -y libssl-dev libcurl4-openssl-dev
```

### Ubuntu 本地授权测试流程

以下流程适合在 Ubuntu 上用自己的电脑先模拟云端授权服务器。

1. 启动授权服务：

   ```bash
   cd /path/to/unitree/qianer_auth_project
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   python scripts/init_db.py
   python scripts/create_activation_key.py --label ubuntu-test --valid-days 30 --max-uses 1
   uvicorn api.main:app --host 0.0.0.0 --port 8000
   ```

2. 在机器人或测试机上查看用于绑定的 MAC 地址：

   ```bash
   ip link show wlp3s0
   ```

   如果实际使用的是其他网卡，例如 `enp3s0`、`wlan0` 或真实机器人上的 `eth0`，后续把 `wlp3s0` 替换成对应网卡名。不要用 `lo`，因为 `00:00:00:00:00:00` 不是可用于授权绑定的真实硬件指纹。

3. 请求激活并保存 license：

   ```bash
   cd /path/to/unitree/unitree_g1
   mkdir -p license

   curl -X POST http://127.0.0.1:8000/v1/activate \
     -H "Content-Type: application/json" \
     -d '{"mac_address":"AA:BB:CC:DD:EE:FF","activation_key":"QE-xxxx"}' \
     | python3 -c 'import sys,json; print(json.dumps(json.load(sys.stdin)["license_content"], indent=4))' \
     > license/qianer_license.lic
   ```

   其中：

   - `AA:BB:CC:DD:EE:FF` 替换为第 2 步查到的 MAC 地址。
   - `QE-xxxx` 替换为第 1 步生成的激活码。
   - 如果授权服务在另一台电脑上，把 `127.0.0.1` 替换为授权服务电脑的局域网 IP。

4. 编译开启 license 验证：

   ```bash
   cd /path/to/unitree/unitree_g1
   mkdir -p build
   cd build
   cmake .. -DENABLE_QIANER_LICENSE_AUTH=ON
   make -j4
   ```

5. 确认控制器 license 配置：

   ```bash
   cd /path/to/unitree/unitree_g1
   cat config/qianer_auth.json
   ```

   Ubuntu 笔记本本地测试时，`iface` 应与第 2 步查看 MAC 的网卡一致，例如：

   ```json
   {
     "cert_path": "../qianer_auth_project/keys/ZJUDES.crt",
     "license_path": "license/qianer_license.lic",
     "iface": "wlp3s0"
   }
   ```

   部署到真实机器人时，把 `iface` 改成机器人实际用于授权绑定的稳定网卡，例如 `eth0`。

6. 运行控制器：

   ```bash
   ./wbc_fsm
   ```

   正常通过时会看到类似输出：

   ```text
   [QianerAuth] License verification is enabled.
   [QianerAuth] License verification passed.
   ```

   如果 license 文件不存在、签名不匹配、MAC 不一致或授权过期，程序会输出失败原因并退出。

### U 盘部署与升级

当前工程提供 `deploy/` 目录，用于生成 U 盘升级包并在机器人端安装、升级和回滚：

```text
deploy/
├── install.sh
├── upgrade.sh
├── rollback.sh
├── start.sh
├── stop.sh
├── qianer-g1.service
└── package_release.sh
```

机器人端默认安装目录：

```text
/opt/qianer/unitree_g1/
├── app -> releases/<version>/
├── releases/
├── shared/
│   ├── config/
│   ├── license/
│   ├── keys/
│   └── logs/
└── scripts/
```

其中 `shared/` 是客户现场数据目录，升级时不会覆盖已有配置和 license：
- `shared/config/`：保存 `qianer_auth.json`、`wbc_dances.json` 等现场配置。
- `shared/license/`：保存 `qianer_license.lic`。
- `shared/keys/`：保存 `ZJUDES.crt` 公钥证书。
- `shared/logs/`：保存运行日志。

打包前先在目标平台完成编译，例如真实机器人 aarch64 环境：

```bash
cd /path/to/unitree_g1
mkdir -p build
cd build
cmake .. -DENABLE_QIANER_LICENSE_AUTH=ON
make -j4
```

生成 U 盘升级包：

```bash
cd /path/to/unitree_g1
DEPLOY_IFACE=eth0 bash deploy/package_release.sh
```

生成结果位于：

```text
dist/
├── qianer_g1_<version>.tar.gz
├── install.sh
└── upgrade.sh
```

把 `dist/` 下的文件复制到 U 盘。首次安装时，在机器人上执行：

```bash
sudo bash /media/unitree/<USB>/install.sh /media/unitree/<USB>/qianer_g1_<version>.tar.gz
```

后续升级时执行：

```bash
sudo bash /media/unitree/<USB>/upgrade.sh /media/unitree/<USB>/qianer_g1_<version>.tar.gz
```

如果机器人还没有 `shared/license/qianer_license.lic`，安装脚本会完成文件部署和 systemd 服务安装，但不会启动控制器。完成授权激活并保存 license 后，再启动：

```bash
sudo systemctl start qianer-g1.service
```

查看状态和日志：

```bash
sudo systemctl status qianer-g1.service
journalctl -u qianer-g1.service -f
```

手动回滚到上一版本：

```bash
sudo bash /opt/qianer/unitree_g1/scripts/rollback.sh
```

如果需要只安装文件、不自动启动服务，可在安装或升级前设置：

```bash
sudo QIANER_NO_AUTO_START=1 bash /media/unitree/<USB>/upgrade.sh /media/unitree/<USB>/qianer_g1_<version>.tar.gz
```

## 配置

配置文件位于 `config/` 目录：
- `wbc.json`：WBC 状态配置
- `loco.json`：运动状态配置
- `fixedpose.json`：固定关节状态配置
- `passive.json`：阻尼状态配置

配置示例（`wbc.json`）：
```json
{
    "model_path": "model/wbc/lafan1_0128_1.onnx",
    "folder_path": "motion_data/lafan1/dance12_binary",
    "enter_idx": 0,
    "pause_idx": 350,
    "safe_projgravity_threshold": 0.5
}
```

## 运行

### 在 Mujoco 仿真中部署

1. 按照 https://github.com/unitreerobotics/unitree_mujoco 的说明安装 Unitree Mujoco

2. 在 `CMakeLists.txt` 中设置 ONNX Runtime 路径：
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)
   ```

3. 在 `controller/src/interface/IOSDK.cpp` 中配置网络接口：
   ```cpp
   ChannelFactory::Instance()->Init(1, "lo"); // lo 用于仿真
   ```

4. 编译项目：
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. 修改unitree_mujoco/config.yaml配置，并启动仿真：
    ```yaml
    robot: "g1"  # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
    robot_scene: "scene_29dof.xml" # Robot scene, /unitree_robots/[robot]/scene.xml 
    domain_id: 1  # Domain id
    interface: "lo" # Interface 
    use_joystick: 0 # 当前控制命令由本控制器进程的终端键盘输入生成
    joystick_type: "xbox" # 仅在重新启用手柄仿真时使用
    joystick_device: "/dev/input/js0" # 可选手柄设备路径
    joystick_bits: 16 # 可选手柄精度配置
    print_scene_information: 1 # Print link, joint and sensors information of robot
    enable_elastic_band: 1 # Virtual spring band, used for lifting h1
    ```
   ```bash
   cd simulate/build
   ./unitree_mujoco
   ```

6. 运行控制器（在新终端中）：
   ```bash
   cd controller/build
   ./wbc_fsm
   ```

### 在真实机器人上部署

1. 将本项目复制到 Unitree G1 机器人 PC2 电脑的 `/home/unitree` 目录下

2. 在 `CMakeLists.txt` 中设置 ONNX Runtime 路径：
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-aarch64-1.22.0)
   ```

3. 在 `controller/src/interface/IOSDK.cpp` 中配置网络接口：
   ```cpp
   ChannelFactory::Instance()->Init(0, "eth0"); // eth0 用于真实机器人
   ```

4. 编译项目：
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. 运行控制器：
   ```bash
   ./wbc_fsm
   ```

## 项目结构

```
controller/
├── config/           # 配置文件
├── include/          # 头文件
│   ├── common/      # 通用工具
│   ├── control/     # 控制组件
│   ├── FSM/         # 状态机状态
│   ├── interface/   # 硬件接口
│   └── message/     # 消息定义
├── src/             # 源文件
│   ├── main.cpp
│   ├── control/
│   ├── FSM/
│   └── interface/
├── model/           # ONNX 模型
├── motion_data/     # 动作参考数据
└── CMakeLists.txt
```

## 控制说明

### 键盘操作指令

当前工程已经将操作输入从宇树遥控器切换为电脑终端键盘。括号中的 `UserCommand` 名称保留了原遥控器命名，是为了兼容状态机内部逻辑。

- **0**：退出程序（`SELECT`）
- **1**：进入固定站立 / 位控准备状态（`START`）
- **2**：进入 AMP 模式（`R2_A`）
- **3**：进入 WBC 全身控制模式（`R1_UP`）
- **4**：进入 WBC Left 状态（`R1_LEFT`）
- **5**：进入 WBC Right 状态（`R1_RIGHT`）
- **6**：选择上一个舞蹈动作
- **7**：选择下一个舞蹈动作
- **8**：打印当前选中的舞蹈动作配置
- **p**：进入 Passive 阻尼保护模式（`L2_B`）
- **[**：在配置帧暂停动作（`R2`）
- **]**：继续动作（`R1`）
- **l**：在当前帧暂停动作（`L2`）
- **b**：从 AMP 返回 Loco（`R2_B`）
- **+ / =**：切换快速模式（`R2_UP`）
- **-**：切换慢速模式（`R2_DOWN`）
- **w / s**：前进 / 后退速度指令
- **a / d**：左右方向修正指令
- **q / e**：左转 / 右转修正指令
- **空格**：清零速度指令

### 操作步骤

1. 运行程序后，机器人处于**阻尼保护模式**
2. 按键盘 **1** 进入固定站立 / 位控准备状态
3. 将机器人悬吊起来（在仿真中默认启用 `enable_elastic_band`，按键盘数字键 **9** 可以松开绑带，再次按下可重新悬吊，数字键 **8** 下放，数字键 **7** 上拉）
4. 按键盘 **2** 进入 AMP Mode，此时松开吊绳
   - 按 **+ / =** 可以进入快速模式（跑步）
   - 按 **-** 可以进入慢速模式（行走）
5. 需要进入 WBC Mode 时，按键盘 **3**
   - 按 **[** 或 **l** 可以暂停动作
   - 按 **]** 可以继续动作

### 舞蹈动作配置

WBC 舞蹈动作和模型统一配置在：

```text
config/wbc_dances.json
```

每个舞蹈 profile 需要配置对应的 ONNX 模型路径和动作 bin 数据目录：

```json
{
  "id": "tiktok",
  "name": "Tiktok Video",
  "model_path": "model/wbc/tiktok_video.onnx",
  "motion_path": "motion_data/lafan1/tiktok_video/converted_bin",
  "start_idx": 0,
  "end_idx": -1,
  "pause_idx": 350,
  "safe_projgravity_threshold": 0.6,
  "debug": true,
  "debug_interval": 50,
  "return_to_amp_blend_frames": 20
}
```

按键 **6 / 7 / 8** 只改变“下一次进入 WBC 使用的舞蹈配置”。WBC 正在跳舞时不会热切换 ONNX 或动作 bin，避免实时控制周期卡顿。

## 许可证

本项目基于 Unitree Robotics SDK2 框架开发。

原始框架：Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. 保留所有权利。

修改和扩展：[ccrpRepo / ZSTU Robotics] © 2026

## 致谢

- 基于 Unitree Robotics SDK2 开发
- 动作数据来自 LAFAN1 数据集
- 使用 ONNX Runtime 进行模型推理
