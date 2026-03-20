# config.json 配置说明

本文件为 TobaccoVision 系统的统一配置入口，各模块在构造时自动从此文件读取参数，缺失的字段会回退到各模块内置的默认值。

---

## system -- 系统全局参数

控制物理模型和数值范围，影响深度图到高程图的转换以及体积计算。

| 键名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `install_height_mm` | float | 2700.0 | 深度相机距传送带面的安装高度（毫米）。用于深度值转物体高度：`h = install_height - depth`。需根据实际安装精确测量。 |
| `belt_speed_m_s` | float | 0.132 | 传送带运行速度（米/秒）。与 `fps` 共同决定每帧对应的物理位移，从而控制 DEM 拼接精度。 |
| `fps` | float | 5.0 | 深度相机的采集帧率（帧/秒）。每帧位移 = `belt_speed_m_s / fps`。 |
| `px_per_m` | int | 100 | DEM 分辨率，每米对应多少像素。值为 100 时，1 个像素代表 10mm x 10mm 的物理面积。 |
| `min_valid_height_mm` | float | 0.0 | 有效高度下限（毫米）。低于此值的像素被视为噪声，不参与缺陷检测的背景均值计算。 |
| `max_valid_height_mm` | float | 2000.0 | 有效高度上限（毫米）。超过此值的像素视为无效测量（如天花板反射），会被替换为安全背景高度。 |
| `material_rho` | float | 150.0 | 物料密度（kg/m³）。用于质量估算：`mass = volume * material_rho`。烟草叶片压实后密度约 150 kg/m³。 |
| `min_object_height_m` | float | 0.02 | 最小有效物体高度（米）。低于此高度的 DEM 像素不计入体积积分，用于过滤空柜底部噪声。 |

---

## mapping -- DEM 拼接参数

控制数字高程模型（DEM）的尺寸和 PLY 点云的导出精度。

| 键名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `dem_rows` | int | 200 | DEM 矩阵行数，对应垂直于传送带方向的物理宽度。`dem_rows / px_per_m` = 传感器覆盖宽度（米）。默认 200px = 2 米。 |
| `dem_cols` | int | 6000 | DEM 矩阵最大列数，对应传送带方向的最大覆盖长度。`dem_cols / px_per_m` = 最大覆盖长度（米）。默认 6000px = 60 米。 |
| `ply_stride` | int | 2 | PLY 点云导出时的降采样步长。步长为 2 时每隔一个像素采样一次，点数减少为原来的 1/4，文件大小同比缩小。设为 1 时导出全密度点云。 |

---

## defect -- 缺陷检测参数

控制背景估计算法和缺陷过滤阈值，直接影响检测灵敏度和误报率。

| 键名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `threshold_mm` | float | 80.0 | 缺陷偏差阈值（毫米）。局部高度与背景的偏差超过此值，才被标记为缺陷候选。值越小越灵敏，误报率越高。 |
| `min_long_mm` | float | 300.0 | 缺陷包围盒长轴最小尺寸（毫米）。小于此尺寸的候选区域被过滤，避免将细小颗粒误报为缺陷。 |
| `min_short_mm` | float | 100.0 | 缺陷包围盒短轴最小尺寸（毫米）。`min_long_mm` 和 `min_short_mm` 需同时满足才上报缺陷。 |
| `margin_x_px` | int | 35 | 水平方向安全边距（像素）。图像左右各裁去此宽度后再检测，排除传送带边缘的遮挡和几何畸变。 |
| `margin_y_px` | int | 45 | 垂直方向安全边距（像素）。图像上下各裁去此高度后再检测。 |
| `bg_kernel_width` | int | 301 | 背景估计 boxFilter 的水平核宽度（像素），**必须为奇数**，配置为偶数时程序自动 +1。核越宽，背景越平滑，对大面积缓坡不敏感；核越窄，对局部突变越灵敏。默认 301px = 约 3 米。 |
| `morph_kernel_size` | int | 5 | 形态学开运算的正方形核大小（像素），**必须为奇数**，配置为偶数时程序自动 +1。用于去除阈值分割后的小噪声点，防止将孤立噪声像素误报为缺陷。 |
| `draw_filtered` | bool | false | 调试开关。设为 `true` 时，在检测结果图上用蓝色框标注被尺寸阈值过滤掉的缺陷候选，方便调试阈值。生产环境应设为 `false`。 |

---

## camera -- 相机内参

用于帧校验和坐标系计算。

| 键名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `fx` | float | 600 | X 方向焦距（像素） |
| `fy` | float | 600 | Y 方向焦距（像素） |
| `cx` | float | 320.0 | 光心 X 坐标（像素） |
| `cy` | float | 180.0 | 光心 Y 坐标（像素） |
| `frame_width` | int | 640 | 输入深度帧宽度（像素）。`analyzeStream()` 用此值校验输入帧的字节大小（必须 = `frame_width * frame_height * 2`）。 |
| `frame_height` | int | 360 | 输入深度帧高度（像素）。 |

---

## python -- Python 环境配置

仅供 `plyToArray()` 使用，指定调用 Python 脚本时的解释器和脚本路径。

| 键名 | 类型 | 说明 |
|---|---|---|
| `python_bin` | string | Python 解释器的**绝对路径**，需包含 open3d 和 numpy 包。推荐使用 conda 环境的完整路径，如 `/home/user/.conda/envs/myenv/bin/python`。 |
| `script_path` | string | `integrated_pipeline_optimized.py` 相对于工作目录（项目根目录）的路径，默认 `api/integrated_pipeline_optimized.py`。 |

---

## 当前配置示例

```json
{
    "system": {
        "install_height_mm": 2700.0,
        "belt_speed_m_s": 0.132,
        "fps": 5.0,
        "px_per_m": 100,
        "min_valid_height_mm": 0.0,
        "max_valid_height_mm": 2000.0,
        "material_rho": 150.0,
        "min_object_height_m": 0.02
    },
    "mapping": {
        "dem_rows": 200,
        "dem_cols": 6000,
        "ply_stride": 2
    },
    "defect": {
        "threshold_mm": 80.0,
        "min_long_mm": 300.0,
        "min_short_mm": 100.0,
        "margin_x_px": 35,
        "margin_y_px": 45,
        "bg_kernel_width": 301,
        "morph_kernel_size": 5,
        "draw_filtered": false
    },
    "camera": {
        "fx": 600,
        "fy": 600,
        "cx": 320.0,
        "cy": 180.0,
        "frame_width": 640,
        "frame_height": 360
    },
    "python": {
        "python_bin": "/home/zsc/.conda/envs/npl_to_list/bin/python",
        "script_path": "api/integrated_pipeline_optimized.py"
    }
}
```
