"""
integrated_pipeline_optimized.py

将 PLY 格式点云文件转换为固定尺寸的一维深度数组。

处理步骤：
  1. 用 Open3D 读取 PLY 点云
  2. XY 坐标原点偏移归零
  3. Y 轴裁剪：保留 y_min ~ y_max 范围内的有效区域（剔除传送带边缘）
  4. 统计离群点滤波（Statistical Outlier Removal）
  5. 二次原点偏移
  6. 将点云投影到 grid_y x grid_x 均匀网格，取每格 Z 均值

命令行用法：
  python integrated_pipeline_optimized.py <ply_path>
  输出：JSON 格式的一维 float 列表（stdout），错误时输出 {"error": "..."}
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


def process_point_cloud(input_file, y_min=0.3, y_max=1.65,
                        nb_neighbors=10, std_ratio=3,
                        grid_y=20, grid_x=450, visualize=False):
    """
    将 PLY 点云转换为一维深度数组。

    Args:
        input_file:   输入 PLY 文件路径
        y_min:        Y 轴裁剪下限（米），默认 0.3
        y_max:        Y 轴裁剪上限（米），默认 1.65
        nb_neighbors: 统计滤波邻居数量，默认 10
        std_ratio:    统计滤波标准差倍数，默认 3
        grid_y:       输出网格行数，默认 20
        grid_x:       输出网格列数，默认 450
        visualize:    是否弹窗可视化各步骤结果，调试用，默认 False

    Returns:
        list[float]: 行优先的一维深度数组，长度 = grid_y * grid_x = 9000
    """

    # 读取点云文件
    pcd = o3d.io.read_point_cloud(input_file)
    if pcd.is_empty():
        raise FileNotFoundError(f"Cannot read: {input_file}")
    if visualize:
        o3d.visualization.draw_geometries([pcd], window_name="1. 原始点云")

    # 转为 numpy 数组（float32 降低内存占用），后续全程向量化操作
    points = np.asarray(pcd.points, dtype=np.float32)

    # 第一次 XY 原点偏移：将点云平移到 XY 原点
    points[:, :2] -= points[:, :2].min(axis=0)

    # Y 轴裁剪：去除烟柜侧边（传送带边缘）的干扰区域
    mask = (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
    points = points[mask]
    if visualize:
        pcd_crop = o3d.geometry.PointCloud()
        pcd_crop.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd_crop], window_name="2. 裁剪Y轴")

    # 统计离群点滤波（最耗时步骤）：去除测量噪声产生的孤立点
    pcd_temp = o3d.geometry.PointCloud()
    pcd_temp.points = o3d.utility.Vector3dVector(points)
    _, ind = pcd_temp.remove_statistical_outlier(nb_neighbors, std_ratio)
    points = points[ind]
    if visualize:
        pcd_clean = o3d.geometry.PointCloud()
        pcd_clean.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd_clean], window_name="3. 去除异常点")

    # 第二次 XY 原点偏移：裁剪和滤波后重新对齐坐标系
    points[:, :2] -= points[:, :2].min(axis=0)

    # 网格映射：将点云投影到 grid_y x grid_x 的均匀网格
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    # 以 Y 方向分辨率作为统一格距
    res = (y.max() - y.min() + 1e-8) / grid_y
    y_idx = np.clip((y / res).astype(np.int32), 0, grid_y - 1)
    x_idx = (x / res).astype(np.int32)

    # 过滤超出 grid_x 范围的点，然后累加 Z 值和计数
    mask = x_idx < grid_x
    depth = np.zeros((grid_y, grid_x), dtype=np.float32)
    counts = np.zeros((grid_y, grid_x), dtype=np.int32)
    np.add.at(depth, (y_idx[mask], x_idx[mask]), z[mask])
    np.add.at(counts, (y_idx[mask], x_idx[mask]), 1)

    # 计算每格平均深度（无点的格子保留为 0）
    np.divide(depth, counts, out=depth, where=counts > 0)
    if visualize:
        plt.figure(figsize=(15, 3))
        plt.imshow(depth, cmap='jet', aspect='auto', origin='lower')
        plt.colorbar(label='Depth')
        plt.title("4. 深度图")
        plt.show()

    return depth.flatten().tolist()


if __name__ == "__main__":
    import sys, json
    if len(sys.argv) < 2:
        print(json.dumps({"error": "usage: script <ply_path>"}))
        sys.exit(1)
    try:
        result = process_point_cloud(sys.argv[1], visualize=False)
        # 输出 JSON 数组到 stdout，由 C++ 的 popen 读取
        print(json.dumps(result))
    except Exception as e:
        print(json.dumps({"error": str(e)}))
