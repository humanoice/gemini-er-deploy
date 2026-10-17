# roboparty_deploy

[![许可证：GPL v3](https://img.shields.io/badge/许可证-GPLv3-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
![Linux](https://img.shields.io/badge/Linux-x86_64%20%7C%20aarch64-orange)

[English](README.md) | [中文](README_CN.md)

## 仓库介绍

`roboparty_deploy` 是面向 RPO 和 RP1 的 ROS 2 部署框架，包含电机驱动、IMU、策略推理和深度感知模块。

仓库采用模块化架构，提供完整的部署代码，支持自定义功能包扩展。

开源地址：[Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy)。维护者：RoboParty；支持渠道：GitHub Issues。

本分支新增 `er_goto`：在提示符下输入“walk to the chair”（走到椅子前）这样的文字指令，机器人借助 RDK X5 主控上的 Google Gemini Robotics-ER 和 Intel RealSense D435i 找到目标物体并走过去。见[走到椅子前](#走到椅子前)。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [RPO 部署](docs/rpo_deploy_CN.md) | RPO 主控连接、环境准备、实时内核、AP 配置、接线、标零及策略和相机启动 |
| RP1 部署 | Coming soon（即将发布） |
| [YAML 配置参考](docs/configuration_CN.md) | 配置文件加载关系、硬件、推理、相机和标零参数，以及排列顺序和约束 |
| [通用操作](docs/operation_CN.md) | 构建、ROS 自动检测、日志、手柄和服务 |
| [Python SDK](docs/python_sdk_CN.md) | IMU、电机、机器人接口，以及示例脚本适用范围 |
| [走向物体（er_goto，英文）](er_goto/README.md) | Gemini Robotics-ER 走向物体节点：工作原理、参数、安全与测试计划 |

## 项目架构

| 路径 | 职责 |
| --- | --- |
| `src/motors` | DM、EVO、LRO、RP、XYN 电机驱动与通信 |
| `src/imu` | HIPNUC、MCT7123 IMU 驱动 |
| `src/inference` | 机器人接口、ONNX 策略推理、观测组装和动作数据读取 |
| `src/inference/robots` | 各机器人的硬件配置、策略参数、模型与动作数据 |
| `src/camera` | RealSense 接入、深度处理与观测发布 |
| `er_goto` | 用 Gemini Robotics-ER 在 RealSense 图像中定位物体并走过去；发布 `/cmd_vel` |
| `tools` | 环境检测及机器人、相机、`er_goto` 启动工具 |
| `scripts` | Python SDK 示例和电机标零工具 |
| `docs` | 环境、部署、配置与接口文档 |

推理模块通过机器人接口读取电机和 IMU 状态，将策略输出转换为关节目标，由电机驱动执行。相机模块独立处理深度图，并通过 ROS 话题向推理模块提供感知观测。`er_goto` 与推理模块并行运行，只在 `/cmd_vel` 上发送速度指令；操作员在手柄上按 Y 后，`default` 策略才会执行这些指令。

## 走到椅子前

`er_goto` 边走边看。Gemini Robotics-ER 在后台线程中指出相机图像里椅子的位置，本地代码结合对齐的深度图，把该点换算为方位角和距离。两次回答之间，机器人朝最新结果继续行走，并用航位推算修正拍照之后自身的移动。只有在确认到达、结果过旧或前方受阻需要重新观察、以及搜索目标时才会停下，最终停在椅子前约 0.8 m 处。

### RDK X5 一次性准备

RealSense SDK 需已安装。然后执行：

```bash
git submodule update --init --recursive        # 包含 RealSense ROS 驱动
sudo apt install -y python3-opencv python3-numpy python3-pytest
pip3 install --user google-genai
cp .env.example .env                           # 然后在 .env 中填写 GEMINI_API_KEY=...
```

不要使用 `pip install opencv-python`：它会安装 numpy 2，导致 ROS Humble 的 Python 包无法使用。`.env` 已被 git 忽略。

### 操作步骤

1. 将机器人放在空旷区域，椅子距离 2-3 m。椅子不必在视野内，机器人会原地转动寻找。D435i 需接在 USB 3 接口上。
2. 终端 1：以 `default` 策略启动机器人，然后像平常一样在手柄上启动策略（见[通用操作](docs/operation_CN.md)），让机器人保持平衡。

   ```bash
   ./tools/start_robot.sh
   ```

3. 终端 2：启动 `er_goto`。

   ```bash
   ./tools/start_er_goto.sh
   ```

   脚本会构建 `er_goto`，在 `er_camera_session` screen 会话中启动相机，检查相机数据流和 `inference_node`，然后启动节点并显示 `er_goto>` 提示符。
4. 脚本打印交接提示后，按手柄 **Y** 键。按 Y 切换为 `/cmd_vel` 控制之前，机器人不会执行 `er_goto` 的指令。
5. 输入要去的地方，可以是完整英文指令，也可以只写物体名称：

   ```
   er_goto> go to the black chair
   er_goto> walk to the lady
   er_goto> stop
   er_goto> quit
   ```

   机器人转向目标，并在 Gemini 持续观察的同时前进。它会停在目标前约 0.8 m 处，输出 `Arrived at "black chair"`，然后等待下一条指令。若转动约一整圈仍未找到目标，节点会放弃、停止机器人并等待下一条指令。机器人行走途中输入的新指令会替换当前目标：机器人先停下，寻找新目标，再走过去。`stop` 让机器人停下并等待；`quit` 或 Ctrl+D 让机器人停下并退出。
6. 再按一次 **Y**，将控制权交回手柄。

整个过程中操作员必须手持手柄。**Y** 立即收回控制权，**B** 暂停推理，在终端 2 按 Ctrl+C 会让机器人停下并退出。如果 `er_goto` 被强制结束而没有清理，机器人会保持最后一条 `/cmd_vel` 指令，此时只能靠 Y 收回控制。相机画面会发送到 Google Gemini API。

常用选项：

| 命令 | 作用 |
| --- | --- |
| `./tools/start_er_goto.sh walk to the chair` | 先执行这条指令，再显示提示符 |
| `./tools/start_er_goto.sh --dry-run` | 发布到 `/cmd_vel_test` 而非 `/cmd_vel`；无需启动机器人 |
| `./tools/start_er_goto.sh --debug-images` | 将每帧相机图像连同 Gemini 标注点保存到 `~/er_goto_debug/<时间>-<目标>/` |
| `./tools/start_er_goto.sh --restart-camera` | 运行前重启相机会话 |
| `screen -S er_camera_session -X quit` | 停止相机（脚本退出后相机仍保持运行） |

不要同时运行 `./tools/start_camera.sh`，它会以 parkour 配置打开同一台相机。首次地面测试前，请按 [er_goto/README.md](er_goto/README.md#test-plan) 中的分阶段测试计划进行：单元测试、照片测试、相机空跑，然后吊装测试。

## 许可证

除另有说明外，本仓库中由 RoboParty 创作的原创内容依据 [GNU General Public License v3.0 only](LICENSE)（`GPL-3.0-only`）授权。

Git 子模块、已打包二进制文件及其他第三方资料继续遵循其各自的版权和许可证条款；仓库级 GPLv3 声明不会替代这些条款。当前许可证清单和再分发说明见 [LICENSES.md](LICENSES.md)。
