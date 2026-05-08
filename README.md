# WBC_Deploy Controller

Whole-Body Control deployment system for humanoid robots using reinforcement learning and motion tracking.

English | [中文](README_zh.md)

## Features

- **State Machine Control**: Multiple FSM states including Passive, Loco (locomotion), and WBC (whole-body control)
- **Motion Tracking**: Real-time tracking of LAFAN1 motion dataset retargeted for Unitree G1 humanoid robots
- **ONNX Runtime**: Fast inference with ONNX models
- **Configurable**: JSON-based configuration for easy parameter tuning

## Prerequisites

- CMake >= 3.14
- C++17 compiler
- CUDA
- Required libraries:
  - unitree_sdk2
  - **ONNX Runtime 1.22.0** (see installation below)
  - Eigen3
  - nlohmann_json >= 3.7.3
  - Boost

### Installing ONNX Runtime

Download and extract ONNX Runtime 1.22.0 to the `controller/` directory:

**For x64 (Simulation):**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

**For aarch64 (Real Robot):**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

## Building

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

## Configuration

Configuration files are located in `config/`:
- `wbc.json`: WBC state configuration
- `loco.json`: Locomotion state configuration
- `fixedpose.json`: FixedStand state configuration
- `passive.json`: Passive state configuration

Example configuration (`wbc.json`):
```json
{
    "model_path": "model/wbc/lafan1_0128_1.onnx",
    "folder_path": "motion_data/lafan1/dance12_binary",
    "enter_idx": 0,
    "pause_idx": 350,
    "safe_projgravity_threshold": 0.5
}
```

## Running

### Deploy on Mujoco Simulation

1. Install Unitree Mujoco following the instructions at https://github.com/unitreerobotics/unitree_mujoco

2. Set the ONNX Runtime path in `CMakeLists.txt`:
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)
   ```

3. Configure the network interface in `controller/src/interface/IOSDK.cpp`:
   ```cpp
   ChannelFactory::Instance()->Init(1, "lo"); // lo for simulation
   ```

4. Build the project:
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. Edit unitree_mujoco/config.yaml and Start the simulation:
    ```yaml
    robot: "g1"  # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
    robot_scene: "scene_29dof.xml" # Robot scene, /unitree_robots/[robot]/scene.xml 
    domain_id: 1  # Domain id
    interface: "lo" # Interface 
    use_joystick: 0 # Keyboard input is handled by this controller process
    joystick_type: "xbox" # Optional only when using joystick simulation
    joystick_device: "/dev/input/js0" # Optional joystick device path
    joystick_bits: 16 # Optional joystick accuracy setting
    print_scene_information: 1 # Print link, joint and sensors information of robot
    enable_elastic_band: 1 # Virtual spring band, used for lifting h1
    ```

   ```bash
   cd simulate/build
   ./unitree_mujoco
   ```

6. Run the controller (in a new terminal):
   ```bash
   cd controller/build
   ./wbc_fsm
   ```

### Deploy on Real Robot

1. Copy this project to `/home/unitree` on the Unitree G1 robot's PC2 computer

2. Set the ONNX Runtime path in `CMakeLists.txt`:
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-aarch64-1.22.0)
   ```

3. Configure the network interface in `controller/src/interface/IOSDK.cpp`:
   ```cpp
   ChannelFactory::Instance()->Init(0, "eth0"); // eth0 for real robot
   ```

4. Build the project:
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. Run the controller:
   ```bash
   ./wbc_fsm
   ```

## Controls

### Keyboard Commands

This project currently uses terminal keyboard input instead of the Unitree wireless controller. The `UserCommand` names in parentheses are kept for compatibility with the existing FSM logic.

- **0**: Exit program (`SELECT`)
- **1**: Enter fixed-stand / position-control preparation state (`START`)
- **2**: Enter Loco mode (`R2_A`)
- **3**: Enter WBC mode (`R1_UP`)
- **4**: Enter WBC Left state (`R1_LEFT`)
- **5**: Enter WBC Right state (`R1_RIGHT`)
- **p**: Enter Passive damping state (`L2_B`)
- **[**: Pause motion at the configured reference frame (`R2`)
- **]**: Resume motion (`R1`)
- **l**: Pause motion at the current reference frame (`L2`)
- **b**: Return from AMP to Loco (`R2_B`)
- **+ / =**: Switch to high-speed mode (`R2_UP`)
- **-**: Switch to low-speed mode (`R2_DOWN`)
- **w / s**: Forward / backward velocity command
- **a / d**: Left / right correction command
- **q / e**: Left / right yaw correction command
- **Space**: Clear velocity command

### Operation Procedure

1. After starting the program, the robot enters **Damping Protection Mode**
2. Press keyboard **1** to enter fixed-stand / position-control preparation state
3. Suspend the robot (In simulation, `enable_elastic_band` is enabled by default. Press keyboard **9** to release the band, press again to re-suspend. Press **8** to lower, **7** to raise)
4. Press keyboard **2** to enter Loco(AMP) Mode, then release the suspension band
   - Press **+ / =** to enter high-speed mode (running)
   - Press **-** to enter low-speed mode (walking)
5. Press keyboard **3** to enter WBC (Whole-Body Control) Mode
   - Press **[** or **l** to pause the motion
   - Press **]** to resume the motion

## Project Structure

```
controller/
├── config/           # Configuration files
├── include/          # Header files
│   ├── common/      # Common utilities
│   ├── control/     # Control components
│   ├── FSM/         # State machine states
│   ├── interface/   # Hardware interfaces
│   └── message/     # Message definitions
├── src/             # Source files
│   ├── main.cpp
│   ├── control/
│   ├── FSM/
│   └── interface/
├── model/           # ONNX models
├── motion_data/     # Motion reference data
└── CMakeLists.txt
```

## License

This project is based on Unitree Robotics SDK2 framework.

Original framework: Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.

Modified and extended by [ccrpRepo / ZSTU Robotics] © 2026

## Acknowledgments

- Based on Unitree Robotics SDK2
- Motion data from LAFAN1 dataset
- ONNX Runtime for model inference
