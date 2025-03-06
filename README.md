# armor_detector_openvino

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_openvino/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_openvino/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. 项目介绍

- [armor_detector_openvino](./armor_detector_openvino/): 基于 @Ericsii 开源项目 [rm_vision](https://github.com/Ericsii/rm_vision) OpenVINO 装甲板检测功能包

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Install [OpenVINO 2023](https://docs.openvino.ai/2025/get-started/install-openvino.html?PACKAGE=OPENVINO_BASE&VERSION=v_2023_3_0&OP_SYSTEM=LINUX&DISTRIBUTION=APT)

### 2.2 Create Workspace

```bash
sudo apt install git-lfs
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_openvino.git src/armor_detector_openvino
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2.4 Running

```bash
ros2 launch armor_detector_openvino armor_detector_launch.py
```
