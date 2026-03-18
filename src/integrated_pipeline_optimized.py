import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


def process_point_cloud(input_file, y_min=0.3, y_max=1.65, nb_neighbors=10, std_ratio=3, grid_y=20, grid_x=450, visualize=False):
    """
    将PLY点云转换为一维深度数组（优化版）

    优化点：
    1. 减少点云对象转换，直接操作numpy数组
    2. 使用float32降低内存占用
    3. 向量化所有可能的操作
    4. 减少中间变量拷贝

    Args:
        input_file: 输入PLY文件路径
        y_min: Y轴裁剪最小值
        y_max: Y轴裁剪最大值
        nb_neighbors: 异常点检测邻居数量
        std_ratio: 异常点检测标准差倍数
        grid_y: 网格行数
        grid_x: 网格列数
        visualize: 是否可视化

    Returns:
        list: 行优先一维深度数组
    """

    # 读取点云
    pcd = o3d.io.read_point_cloud(input_file)
    if pcd.is_empty():
        raise FileNotFoundError(f"Cannot read: {input_file}")
    if visualize:
        o3d.visualization.draw_geometries([pcd], window_name="1. 原始点云")

    # 转为numpy数组，后续全部用numpy操作
    points = np.asarray(pcd.points, dtype=np.float32)

    # 第一次原点偏移
    points[:, :2] -= points[:, :2].min(axis=0)

    # 裁剪Y轴（向量化操作）
    mask = (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
    points = points[mask]
    if visualize:
        pcd_crop = o3d.geometry.PointCloud()
        pcd_crop.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd_crop], window_name="2. 裁剪Y轴")

    # 去除异常点（最耗时操作）
    pcd_temp = o3d.geometry.PointCloud()
    pcd_temp.points = o3d.utility.Vector3dVector(points)
    _, ind = pcd_temp.remove_statistical_outlier(nb_neighbors, std_ratio)
    points = points[ind]
    if visualize:
        pcd_clean = o3d.geometry.PointCloud()
        pcd_clean.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd_clean], window_name="3. 去除异常点")

    # 第二次原点偏移
    points[:, :2] -= points[:, :2].min(axis=0)

    # 网格映射（向量化）
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    res = (y.max() - y.min() + 1e-8) / grid_y
    y_idx = np.clip((y / res).astype(np.int32), 0, grid_y - 1)
    x_idx = (x / res).astype(np.int32)

    # 过滤并累加
    mask = x_idx < grid_x
    depth = np.zeros((grid_y, grid_x), dtype=np.float32)
    counts = np.zeros((grid_y, grid_x), dtype=np.int32)
    np.add.at(depth, (y_idx[mask], x_idx[mask]), z[mask])
    np.add.at(counts, (y_idx[mask], x_idx[mask]), 1)

    # 计算平均深度
    np.divide(depth, counts, out=depth, where=counts > 0)
    if visualize:
        plt.figure(figsize=(15, 3))
        plt.imshow(depth, cmap='jet', aspect='auto', origin='lower')
        plt.colorbar(label='Depth')
        plt.title("4. 深度图")
        plt.show()

    return depth.flatten().tolist()


if __name__ == "__main__":
    import time
    start = time.time()
    result = process_point_cloud("Global_Stitched_20240101.ply", visualize=True)
    elapsed = time.time() - start
    print(f"处理完成，耗时: {elapsed:.2f}秒")
    print(f"输出长度: {len(result)}")
