import numpy as np
import os


def save_unitree_bin(filename, array_data):
    """
    将 numpy 数组严格按照 Unitree C++ 底层的私有协议打包成 .bin
    """
    with open(filename, 'wb') as f:
        # 1. 写入彩蛋魔数 "NPZ\0" (5918798)
        f.write(np.array([5918798], dtype=np.int32).tobytes())

        # 2. 写入维度数量 (ndims)
        f.write(np.array([array_data.ndim], dtype=np.int32).tobytes())

        # 3. 写入每个维度的大小 (shape)
        # 例如 joint_pos 会写入 1618 和 29
        f.write(np.array(array_data.shape, dtype=np.int32).tobytes())

        # 4. 判断数据类型并写入 字节数(bytes) 和 类型ID(type_char)
        if array_data.dtype == np.float32:
            bytes_per_elem = 4
            type_char = ord('f')  # ASCII 102
        elif array_data.dtype == np.float64:
            # 强制转为 float32 以防万一，C++ 底层只认 float32
            array_data = array_data.astype(np.float32)
            bytes_per_elem = 4
            type_char = ord('f')
        elif array_data.dtype == np.int64:
            bytes_per_elem = 8
            type_char = ord('l')  # ASCII 108
        elif array_data.dtype == np.int32:
            bytes_per_elem = 4
            type_char = ord('i')  # ASCII 105
        else:
            raise ValueError(f"不支持的数据类型: {array_data.dtype}")

        f.write(np.array([bytes_per_elem, type_char],
                dtype=np.int32).tobytes())

        # 5. 写入真实的矩阵数据 (C-contiguous 内存顺序)
        f.write(array_data.tobytes())


# ==========================================
# 批量转换执行逻辑
# ==========================================

# 设置你的 npy 所在的绝对路径
base_dir = "/home/simon/Code/unitree_g1/wbc_fsm/motion_data/lafan1/gangnam_style/"
output_dir = os.path.join(base_dir, "converted_bin")

# 创建输出文件夹
os.makedirs(output_dir, exist_ok=True)

# 你的 7 个文件
npy_files = [
    "fps.npy",
    "joint_pos.npy",
    "joint_vel.npy",
    "body_pos_w.npy",
    "body_quat_w.npy",
    "body_lin_vel_w.npy",
    "body_ang_vel_w.npy"
]

print("🚀 开始将江南Style数据转换为 Unitree G1 专属 .bin 格式...\n")

for npy_name in npy_files:
    full_npy_path = os.path.join(base_dir, npy_name)

    if os.path.exists(full_npy_path):
        # 1. 读取 npy
        data = np.load(full_npy_path)

        # 2. 强制类型对齐 (极其重要)
        if "fps" in npy_name:
            data = data.astype(np.int64)
        else:
            data = data.astype(np.float32)

        # 3. 构造输出路径
        bin_name = npy_name.replace(".npy", ".bin")
        full_bin_path = os.path.join(output_dir, bin_name)

        # 4. 执行伪装与写入
        save_unitree_bin(full_bin_path, data)

        print(f"✅ [成功] {npy_name} -> {bin_name}")
        print(f"   |- 形状写入: {data.shape}")
        print(f"   |- 类型写入: {data.dtype}\n")
    else:
        print(f"⚠️ [错误] 找不到文件: {full_npy_path}\n")

print(f"🎉 全部转换完毕！\n请前往目录提取你的成果: {output_dir}")
