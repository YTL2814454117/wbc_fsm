import numpy as np
import os

# 直接写死电脑上的绝对路径，注意最后要加斜杠 /
# base_dir = "/home/simon/Code/unitree_g1/wbc_fsm/motion_data/lafan1/gangnam_style/converted_bin/"
# base_dir = "/home/simon/Code/unitree_g1/wbc_fsm/motion_data/lafan1/dance12/"
base_dir = "/home/simon/Code/unitree_g1/wbc_fsm/motion_data/lafan1/tiktok_video/converted_bin/"

bin_files = [
    base_dir + "fps.bin",
    base_dir + "joint_pos.bin",
    base_dir + "joint_vel.bin",
    base_dir + "body_pos_w.bin",
    base_dir + "body_quat_w.bin",
    base_dir + "body_lin_vel_w.bin",
    base_dir + "body_ang_vel_w.bin"
]

for filename in bin_files:
    if os.path.exists(filename):
        # 核心：以 32位浮点数 (float32) 的格式读取整个二进制文件
        if filename.endswith("fps.bin"):
            # 帧率文件通常只有一个数值，直接读取一个 int32
            data = np.fromfile(filename, dtype=np.int32)
        else:
            data = np.fromfile(filename, dtype=np.float32)

        print(f"========== 分析文件: {filename} ==========")
        print(f"1. 数据总长度 (包含的数字个数): {data.size}")

        # 打印前 10 个数据看看内部长什么样
        print(f"2. 前 10 个数值: {data[:10]}")

        # 针对不同文件的结构推演
        if filename.endswith("fps.bin"):
            print(f"💡 结构推测: 这是帧率文件，包含一个整数值，表示每秒的帧数。")

        elif filename.endswith("joint_pos.bin") or filename.endswith("joint_vel.bin"):
            # 假设 G1 机器人有 29 个自由度 (DOF)
            num_dof = 29
            frames = data.size // num_dof
            print(f"💡 结构推测: 若机器人有 {num_dof} 个关节，该动作包含 {frames} 帧。")
            # 如果你想把它恢复成原本的二维矩阵结构：
            # matrix_data = data.reshape((frames, num_dof))

        elif filename.endswith("body_pos_w.bin") or filename.endswith("body_lin_vel_w.bin") or filename.endswith("body_ang_vel_w.bin"):
            # 空间位置和速度通常是 X, Y, Z 三个维度
            frames = data.size // 3
            print(f"💡 结构推测: 这是三维空间数据 (X, Y, Z)，共 {frames} 帧。")

        elif filename.endswith("body_quat_w.bin"):
            # 四元数包含 W, X, Y, Z 四个维度
            frames = data.size // 4
            print(f"💡 结构推测: 这是四元数姿态数据 (W, X, Y, Z)，共 {frames} 帧。")

        print("\n")
    else:
        print(f"[错误] 当前目录下找不到文件: {filename}\n")
