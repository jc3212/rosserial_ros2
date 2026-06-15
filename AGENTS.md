# AGENTS.md - rosserial_ros2 项目指南

## 项目概述

这是一个 ROS2 上的 **rosserial 服务端**移植，用于在 ROS2 上复用 ROS1 rosserial 客户端设备（Arduino 等），实现 ROS1 到 ROS2 的无感切换。

## 包结构

```
src/
├── rosserial_msg/          # 自定义ROS2消息定义
│   ├── msg/TopicInfo.msg  # 话题注册消息（topic_id, topic_name, message_type等）
│   └── CMakeLists.txt
│
└── rosserial_server/       # 主实现（C++）
    ├── include/rosserial_server/
    │   ├── session/          # 串口会话管理
    │   ├── transport/        # 异步串口读写
    │   ├── protocol/         # 协议解析与序列化
    │   └── ros_adapter/      # ROS消息适配器
    ├── src/serial_node.cpp   # 入口点
    └── CMakeLists.txt
```

## 技术架构

```
Arduino客户端 ──串口──> rosserial_server ──> ROS2话题
                       │
                 Boost.ASIO 异步串口
                       │
              [协议层] rosserial协议解析/打包
                       │         │
                       │    0xFF 0xFE | length | checksum | topic_id | data | checksum
                       │
              [序列化层] ROS1紧密型 ↔ DDS CDR格式转换
```

### 核心组件

| 组件 | 位置 | 功能 |
|------|------|------|
| SerialDrive | session/serial_session.hpp | 串口会话管理，数据收发 |
| AsyncReadBuffer | transport/async_transport.hpp | 异步读取缓冲区 |
| AsyncWriteBuffer | transport/async_transport.hpp | 异步写入缓冲区 |
| Ros_Stream | protocol/serial_stream.hpp | 紧密型数据流操作 |
| Stream_Deserialization | ros_adapter/ros_message_adapter.hpp | ROS1/ROS2消息转换 |
| Serial_Publisher | ros_adapter/topic_handle.hpp | 发布者管理 |
| Serial_Subscriber | ros_adapter/topic_handle.hpp | 订阅者管理 |

## 构建与运行

### 构建
```bash
cd /home/chen/Program/rosserial_ros2_ws
colcon build
```

### 运行
```bash
source install/setup.sh
ros2 run rosserial_server serial_node
```

### 动态参数
```bash
ros2 param set /serial_node port /dev/ttyUSB0
ros2 param set /serial_node baud 115200
```

或直接：
```bash
ros2 run rosserial_server serial_node --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

### 默认参数
- port: `/tmp/ttyV0`
- baud: `115200`

## 已实现功能

1. 端口号和波特率动态修改
2. 客户端数据包校验与解析
3. 紧密序列化流反序列化 → DDS格式序列化
4. DDS格式反序列化 → 紧密型数据映射
5. GenericPublisher/GenericSubscription 动态话题创建

## 待实现功能（按优先级）

1. **服务端紧密型数据流打包发送** - 将ROS2消息转换为rosserial协议格式
2. **动态发布者/订阅者管理** - 支持运行时创建/销毁话题
3. **服务和动作处理逻辑** - 支持ROS服务/Action
4. **多线程性能优化** - 当前为单线程轮询

## 协议格式

### rosserial 数据包结构
```
| 0xFF | 0xFE | length(16bit) | length_checksum | topic_id(16bit) | data[n] | data_checksum |
```

### TopicInfo 消息结构
```
topic_id       # 话题ID
topic_name     # 话题名
message_type   # 消息类型（如 sensor_msgs/msg/Imu）
md5sum         # 消息类型MD5
buffer_size    # 缓冲区大小
```

### 保留的 topic_id
- `0`: PUBLISHER（发布者注册）
- `1`: SUBSCRIBER（订阅者注册）
- `2`: SERVICE_SERVER
- `4`: SERVICE_CLIENT
- `6`: PARAMETER_REQUEST
- `7`: LOG
- `10`: TIME
- `11`: TX_STOP

## 关键代码路径

- 入口点: `src/rosserial_server/src/serial_node.cpp:40`
- 串��配置: `src/rosserial_server/include/rosserial_server/session/serial_session.hpp:36`
- 协议解析: `src/rosserial_server/include/rosserial_server/session/serial_session.hpp:175-296`
- 消息序列化: `src/rosserial_server/include/rosserial_server/ros_adapter/ros_message_adapter.hpp:29`

## 测试相关

- 测试用话题: `my_imu` (sensor_msgs/msg/Imu)
- 测试用 topic_id: `1`

## 开发注意事项

1. **dlopen 加载类型支持库** - 动态加载消息包的 `lib<pkg>__rosidl_typesupport_introspection_cpp.so`
2. **字符串长度限制** - 最大 400 字节，防止异常
3. **不支持数组/序列** - 当前版本不支持数组类型字段
4. **header 处理** - 自动跳过 std_msgs/Header 的 seq 字段
5. **字节序** - 小端序（与 x86/ARM 一致）

## 常用命令

```bash
# 查看话题
ros2 topic list

# 监听话题
ros2 topic echo /my_imu

# 发送测试消息
ros2 topic pub /my_imu sensor_msgs/msg/Imu '{...}'

# 查看参数
ros2 param list

# 调试日志
ros2 run rosserial_server serial_node --ros-args -r __log_level:=debug
```