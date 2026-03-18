# 测试程序说明

## 概述

本目录包含ZMQ消息接收端测试程序，用于验证烟草视觉检测系统的消息发布功能。

## 程序列表

### 1. zmq_receiver.cpp
**功能**: 通用ZMQ消息接收端

**订阅主题**:
- `DEFECTS` - 缺陷检测结果
- `SUMMARY` - 批次汇总信息
- `DEPTH_ARRAY` - 深度数组数据

**输出信息**:
- 缺陷详情（位置、类型、尺寸）
- 体积、质量统计
- 深度数组统计（范围、平均值、非零点数、首尾元素）

**编译运行**:
```bash
g++ -std=c++17 zmq_receiver.cpp -o zmq_receiver -lzmq -I../include
./zmq_receiver
```

### 2. save_depth_array.cpp
**功能**: 保存深度数组到文件

**订阅主题**:
- `DEPTH_ARRAY`

**输出文件**:
- `depth_array_9000.txt` - 包含完整的9000个深度值（20行×450列）

**编译运行**:
```bash
g++ -std=c++17 save_depth_array.cpp -o save_depth_array -lzmq -I../include
./save_depth_array
```

## 消息格式

### DEPTH_ARRAY
```json
{
  "ply_path": "output_ply/Global_Stitched_20240101.ply",
  "grid_y": 20,
  "grid_x": 450,
  "depth_array": [0.442, 0.439, ...]
}
```

### DEFECTS
```json
{
  "ply_path": "output_ply/Global_Stitched_20240103.ply",
  "image_path": "result_images/global_detect.jpg",
  "events": [
    {
      "type": "pit",
      "center_x_m": 1.61,
      "center_y_m": -0.085,
      "dev_mm": -179.34,
      "length_mm": 260.0,
      "width_mm": 590.0
    }
  ]
}
```

### SUMMARY
```json
{
  "total_frames": 32,
  "stitched_length_m": 0.84,
  "total_volume_m3": 1.745,
  "mass_kg": 261.83,
  "ply_path": "output_ply/Global_Stitched_20240101.ply",
  "image_path": "result_images/global_detect.jpg"
}
```

## 使用场景

1. **实时监控**: 使用zmq_receiver查看系统运行状态
2. **数据采集**: 使用save_depth_array保存深度数据用于后续分析
3. **集成测试**: 验证ZMQ消息发布功能是否正常

## 注意事项

- 确保主程序已启动并绑定到tcp://*:5555
- 接收端连接到tcp://localhost:5555
- 深度数组大小固定为9000（20×450）
