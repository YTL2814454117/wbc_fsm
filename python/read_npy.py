import numpy as np
import os

base_dir = "/home/simon/Code/unitree_g1/wbc_fsm/motion_data/lafan1/gangnam_style/"

npy_files = [
    "fps.npy",
    "joint_pos.npy",
    "joint_vel.npy",
    "body_pos_w.npy",
    "body_quat_w.npy",
    "body_lin_vel_w.npy",
    "body_ang_vel_w.npy"
]

print("🕵️ 开始读取并检查 .npy 文件...\n")

for npy_name in npy_files:
    # 🚨 关键修复：拼接完整绝对路径
    full_path = os.path.join(base_dir, npy_name)

    # 注意这里要检查的是 full_path
    if os.path.exists(full_path):
        # 读取的时候也要用 full_path
        data = np.load(full_path)

        print(f"========== 分析文件: {npy_name} ==========")
        print(f"📐 形状 (Shape): {data.shape}")
        print(f"🗂️ 数据类型 (Dtype): {data.dtype}")

        if data.size == 1:
            print(f"👀 数据内容: {data}")
        elif data.ndim == 1:
            print(f"👀 前 5 个数据: {data[:5]}")
        else:
            preview_len = min(data.shape[1], 8)
            print(
                f"👀 第 1 帧前 {preview_len} 个元素: \n   {data[0][:preview_len]} ...")
        print("\n")
    else:
        # 打印 full_path 方便你检查路径到底拼得对不对
        print(f"⚠️ [跳过] 找不到文件: {full_path}\n")
