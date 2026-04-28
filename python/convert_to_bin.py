import os
import numpy as np


def convert_npy_to_bin():
    # 根据图片提取并拼接完整的路径
    # 使用 os.path.expanduser('~') 将 '~' 展开为实际的主文件夹路径 (如 /home/username)
    target_dir = os.path.expanduser(
        "~/Code/unitree_g1/wbc_fsm/motion_data/lafan1/tiktok_video")

    # 检查路径是否存在
    if not os.path.exists(target_dir):
        print(f"错误: 找不到路径 '{target_dir}'。请确保您在该路径下运行或路径拼写正确。")
        return

    print(f"目标目录: {target_dir}")
    print("-" * 40)

    # 获取目录下所有的 .npy 文件
    files = [f for f in os.listdir(target_dir) if f.endswith('.npy')]

    if not files:
        print("该目录下没有找到 .npy 文件。")
        return

    # 遍历并转换文件
    for filename in files:
        npy_path = os.path.join(target_dir, filename)
        bin_filename = filename.replace('.npy', '.bin')
        bin_path = os.path.join(target_dir, bin_filename)

        try:
            # 1. 读取 npy 文件
            data = np.load(npy_path)

            # 2. 转换为二进制并保存
            # tofile() 会将数据以 C 连续 (C-contiguous) 的原始二进制格式写入
            data.tofile(bin_path)

            print(
                f"成功转换: {filename} -> {bin_filename} (形状: {data.shape}, 类型: {data.dtype})")
        except Exception as e:
            print(f"转换 {filename} 失败: {e}")

    print("-" * 40)
    print("全部转换完成！")


if __name__ == "__main__":
    convert_npy_to_bin()
