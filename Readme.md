# rosserial_ros2

基于ROS2的rosserial服务端移植

## 项目简介

此项目是基于ROS2实现的`rosserial`的服务端移植，旨在在ROS2上能复用rosserial客户端部分，通过在ROS2上运行该项目提供的节点实现rosserial在ROS1到ROS2的无感切换。同时也作为个人对深入了解ROS2通信和C++工程能力的提升训练。

## 技术架构

```
Arduino客户端 ──串口──> rosserial_server ──> ROS2话题
                       │
                 Boost.ASIO 异步串口
                       │
              [协议层] rosserial协议解析/打包
                       │
              [序列化层] ROS1紧密型 ↔ DDS CDR格式转换
```

## 已实现功能

1. **端口号和波特率参数动态修改** - 支持运行时通过ros2 param动态修改
2. **客户端数据包校验与解析** - 基于rosserial协议的包解析
3. **客户端发送紧密序列化流的反序列化** - 从ros1紧密型到ROS2 CDR格式
4. **DDS格式序列化流的反序列化** - 从ROS2 CDR到ros1紧密型数据
5. **GenericPublisher/GenericSubscription动态话题创建** - 动态话题注册

## 待实现功能

1. 服务端紧密型数据流的打包和发送
2. 动态发布者和订阅者管理
3. 服务和动作处理逻辑
4. 多线程性能优化

## 构建与运行

```bash
# 构建
cd /home/chen/Program/rosserial_ros2_ws
colcon build

# 运行
source install/setup.sh
ros2 run rosserial_server serial_node

# 参数配置
ros2 run rosserial_server serial_node --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

## 包结构

```
src/
├── rosserial_msg/          # 自定义ROS2消息定义
│   └── msg/TopicInfo.msg
│
└── rosserial_server/       # 主实现（C++）
    ├── session/            # 串口会话管理
    ├── transport/         # 异步串口读写
    ├── protocol/          # 协议解析与序列化
    └── ros_adapter/       # ROS消息适配器
```

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![C++](https://img.shields.io/badge/C++-14-green?logo=c%2B%2B)
