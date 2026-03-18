# 配置文件说明 (config.json)

## system - 系统参数

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| install_height_mm | double | 毫米 | 相机安装高度 |
| belt_speed_m_s | double | 米/秒 | 传送带运行速度 |
| fps | double | 帧/秒 | 相机采集帧率 |
| px_per_m | int | 像素/米 | 像素密度（空间分辨率） |
| min_valid_height_mm | double | 毫米 | 最小有效高度阈值 |
| max_valid_height_mm | double | 毫米 | 最大有效高度阈值 |
| material_rho | double | kg/m³ | 物料密度（用于质量计算） |
| min_object_height_m | double | 米 | 最小物体高度（低于此值视为背景） |

## mapping - 映射参数

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| dem_rows | int | 像素 | 高程图行数（横向分辨率） |
| dem_cols | int | 像素 | 高程图列数（纵向最大长度） |
| ply_stride | int | - | PLY点云采样步长（越大文件越小） |

## defect - 缺陷检测参数

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| threshold_mm | double | 毫米 | 缺陷检测阈值（偏差超过此值视为缺陷） |
| min_long_mm | double | 毫米 | 缺陷最小长轴长度（过滤小目标） |
| min_short_mm | double | 毫米 | 缺陷最小短轴长度（过滤小目标） |
| margin_x_px | int | 像素 | X方向边缘裕度（忽略边缘区域） |
| margin_y_px | int | 像素 | Y方向边缘裕度（忽略边缘区域） |
| bg_kernel_width | int | 像素 | 背景滤波核宽度（越大越平滑） |
| morph_kernel_size | int | 像素 | 形态学操作核大小 |
| draw_filtered | bool | - | 是否绘制过滤后的结果图 |

## runtime - 运行时参数

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| dataset_dir | string | - | 数据集目录路径 |
| watch_mode | bool | - | 监控模式（true: 持续监控, false: 单次处理） |
| scan_interval_ms | int | 毫秒 | 目录扫描间隔 |
| publish_every_frames | int | 帧 | 每N帧发布一次结果 |
| camera_id | int | - | 相机ID标识 |
| batch_id_start | int | - | 批次ID起始值 |
| batch_size | int | 帧 | 批处理大小（每批处理的帧数） |
| use_batch_loader | bool | - | 是否使用多线程批量加载器 |

## zmq - ZeroMQ参数

| 参数 | 类型 | 说明 |
|------|------|------|
| bind | string | ZMQ绑定地址（例如: "tcp://*:5555"） |

## camera - 相机内参

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| fx | double | 像素 | 焦距X分量 |
| fy | double | 像素 | 焦距Y分量 |
| cx | double | 像素 | 主点X坐标 |
| cy | double | 像素 | 主点Y坐标 |

## 调优建议

### 提高检测灵敏度
```json
{
  "defect": {
    "threshold_mm": 60.0,
    "min_long_mm": 200.0,
    "min_short_mm": 80.0
  }
}
```

### 提高处理速度
```json
{
  "runtime": {
    "batch_size": 64,
    "use_batch_loader": true
  },
  "mapping": {
    "ply_stride": 4
  }
}
```

### 提高精度
```json
{
  "system": {
    "px_per_m": 200
  },
  "mapping": {
    "dem_rows": 400,
    "ply_stride": 1
  }
}
```
