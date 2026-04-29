#pragma once

#include <boost/asio/serial_port.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <rcl/publisher.h>
#include <rclcpp/create_generic_subscription.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "rosserial_server/protocol/serial_stream.hpp"
#include "rosserial_server/ros_adapter/ros_message_adapter.hpp"
#include "rosserial_server/transport/async_transport.hpp"
#include <rosserial_msg/msg/detail/topic_info__struct.hpp>

namespace ros2_serial
{

//用于向ros2转发话题数据async_read_buffer
class Serial_Publisher
{

  public:
    Serial_Publisher(rclcpp::Node *node, rosserial_msg::msg::TopicInfo &topic_info)
        : node_(node), topic_info_(topic_info)
    {
        stream_deserialized_ = std::make_shared<Stream_Deserialization>();
        //使用允许在使用时设置类型的“generic”类
        pub_ = node_->create_generic_publisher(topic_info_.topic_name, topic_info_.message_type, qos_);
        stream_deserialized_->init(topic_info_);

    };

    void handle(Ros_Stream &stream)
    {
        //需要确保要的是完整的stream_
        stream_deserialized_->publish(stream, pub_);
    }

  private:
    rclcpp::Node *node_;
    rosserial_msg::msg::TopicInfo topic_info_;
    rclcpp::GenericPublisher::SharedPtr pub_;
    rclcpp::QoS qos_{10};
    std::shared_ptr<Stream_Deserialization> stream_deserialized_;
};

//在写注册包处理逻辑前指定topic_info进行测试
std::shared_ptr<Serial_Publisher> publish_test(rclcpp::Node *node)
{
    rosserial_msg::msg::TopicInfo topic_info;
    topic_info.set__topic_id(1);
    topic_info.topic_name = "my_imu";
    topic_info.message_type = "sensor_msgs/msg/Imu";
    // topic_info.message_type = "geometry_msgs/msg/Twist";

    std::shared_ptr<Serial_Publisher> publisher = std::make_shared<Serial_Publisher>(node, topic_info);
    return publisher;
}

//用于向ros2订阅消息和向客户端写入数据
class Serial_Subscriber
{
  public:
    Serial_Subscriber(rclcpp::Node *node, rosserial_msg::msg::TopicInfo &topic_info,
                      std::shared_ptr<AsyncWriteBuffer<boost::asio::serial_port>> writter)
        : node_(node), topic_info_(topic_info), writter_(writter)
    {
        stream_deserialized_ = std::make_shared<Stream_Deserialization>();
        stream_deserialized_->init(topic_info_);

        sub_ = node_->create_generic_subscription(topic_info_.topic_name, topic_info_.message_type, qos_,
                                                  std::bind(&Serial_Subscriber::callback, this, std::placeholders::_1));
    };

    void callback(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg_ptr)
    {
        auto serial_msg = stream_deserialized_->subscribe(serialized_msg_ptr);
        //打印测试
        Ros_Stream test_stream(serial_msg->get_data(), serial_msg->get_write_len());
        // std::cout << test_stream;
        //将数据写入到下位机
        writter_->write_to_serial(serial_msg, topic_info_.topic_id);
    }

  private:
    rclcpp::Node *node_;
    rosserial_msg::msg::TopicInfo topic_info_;
    rclcpp::GenericSubscription::SharedPtr sub_;
    std::shared_ptr<Stream_Deserialization> stream_deserialized_;
    rclcpp::QoS qos_{10};
    std::shared_ptr<AsyncWriteBuffer<boost::asio::serial_port>> writter_;
};

//测试反序列化ros2消息测试
std::shared_ptr<Serial_Subscriber> subscribe_test(rclcpp::Node *node,
                                                  std::shared_ptr<AsyncWriteBuffer<boost::asio::serial_port>> writter)
{
    rosserial_msg::msg::TopicInfo topic_info;
    topic_info.set__topic_id(1);
    topic_info.topic_name = "my_imu";
    topic_info.message_type = "sensor_msgs/msg/Imu";
    std::shared_ptr<Serial_Subscriber> subscriber = std::make_shared<Serial_Subscriber>(node, topic_info, writter);
    return subscriber;
}



} // namespace ros2_serial