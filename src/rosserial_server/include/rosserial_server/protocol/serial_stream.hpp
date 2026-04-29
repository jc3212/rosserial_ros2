#pragma once
#include <cstddef>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include "rosserial_msg/msg/topic_info.hpp"
#include "rosserial_msg/msg//detail/topic_info__struct.hpp"

namespace ros2_serial {
//从紧密型数据中读取各个类型的流式操作符
class Ros_Stream
{
  public:
    Ros_Stream(const uint8_t *buffer, const size_t length)
        : head_(buffer), length_(length), cur_(buffer), end_(buffer + length){};
    Ros_Stream(const size_t length) : head_(nullptr), length_(length), cur_(nullptr), end_(nullptr){};
    template <typename T> Ros_Stream &operator>>(T &value)
    {
        if (head_ == nullptr)
        {
            throw std::out_of_range("The accessed Stream has not been initialized!");
        }
        //获取要读取的类型长度
        size_t value_length = sizeof(value);
        RCLCPP_DEBUG(rclcpp::get_logger("async_read"), "Request bytes : %d , Remain bytes : %d", value_length,
                     this->get_length());

        //判断缓冲区中剩余长度是否足够
        if (cur_ + value_length <= end_)
        {
            //小端序
            std::memcpy(&value, cur_, value_length);
            cur_ += value_length;
        }
        else
        {
            throw std::out_of_range("Serialization data access out of bounds!");
        }

        return *this;
    }
    Ros_Stream &operator>>(std::string &val_string)
    {
        if (head_ == nullptr)
        {
            throw std::out_of_range("The accessed Stream has not been initialized!");
        }
        size_t len = val_string.size();
        if (!len)
        {
            throw std::out_of_range("The length of the string to be written is undefined.");
        }
        if (get_length() < len)
        {
            throw std::out_of_range("Serialization data access out of bounds!");
        }
        else
        {
            memcpy(&val_string[0], cur_, len);
            cur_ += len;
        }
        return *this;
    }
    Ros_Stream &operator>>(Ros_Stream &write_stream)
    {

        size_t request_length = write_stream.get_length();

        if (head_ == nullptr)
        {
            throw std::out_of_range("The accessed Stream has not been initialized!");
        }
        //获取要读取的类型长度
        RCLCPP_DEBUG(rclcpp::get_logger("async_read"), "Request bytes : %zu , Remain bytes : %zu", request_length,
                     this->get_length());

        //判断缓冲区中剩余长度是否足够
        if (cur_ + request_length <= end_)
        {
            //小端序
            write_stream.head_ = cur_;
            write_stream.length_ = request_length;
            write_stream.cur_ = cur_;
            write_stream.end_ = cur_ + request_length;
            cur_ += request_length;
        }
        else
        {
            throw std::out_of_range("Serialization data access out of bounds!");
        }

        return *this;
    }
    //从消息结构体中读取string,此时的string不再是只有字符串
    bool read_str_from_struct(std::string &val_string)
    {
        size_t size_string = sizeof(std::string);
        if (head_ == nullptr)
        {
            throw std::out_of_range("The accessed Stream has not been initialized!");
        }
        if (get_length() < size_string)
        {
            throw std::out_of_range("Attempted to read std::string object beyond ROS 2 struct bounds!");
            return false;
        }
        else
        {
            const std::string *str_ptr = reinterpret_cast<const std::string *>(cur_);
            val_string = *str_ptr;

            cur_ += size_string;
        }
        return true;
    }
    size_t get_length() const
    {
        if (head_ == nullptr && end_ == nullptr)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("async_read"),
                         "The pointer has not been initialized yet! The returned value is the initialized length.");
            return get_total_length();
        }
            return static_cast<int>(end_ - cur_);
    }
    size_t get_total_length() const
    {
        return length_;
    }
    const uint8_t *get_head() const
    {
        return head_;
    }
    const uint8_t *get_cur() const
    {
        return cur_;
    }

  private:
    const uint8_t *cur_;
    const uint8_t *head_;
    const uint8_t *end_;
    size_t length_;
};
//用于打印测试
std::ostream &operator<<(std::ostream &os, const Ros_Stream &stream)
{
    const size_t length = stream.get_total_length();
    const uint8_t *data = stream.get_head();

    //仅备份当前流的标志位和填充字符，极其轻量
    std::ios_base::fmtflags old_flags = os.flags();
    char old_fill = os.fill();

    // 设置流格式为十六进制，并用 0 填充
    os << std::hex << std::setfill('0');

    os << "[ Length: " << std::dec << length << " bytes ]\n[ "; // 先用十进制打印总长度，再开始数据

    // 切回十六进制
    os << std::hex;
    for (size_t i = 0; i < length; i++)
    {
        //每次输出完一个字节后，加上一个空格！
        os << "0x" << std::setw(2) << static_cast<int>(data[i]) << " ";

        // 每 16 个字节换一次行
        if ((i + 1) % 16 == 0 && (i + 1) != length)
        {
            os << "\n  ";
        }
    }
    os << "]\n";

    //恢复流的原始状态
    os.flags(old_flags);
    os.fill(old_fill);

    return os;
}
bool excessive_testing(uint32_t len)
{
    if (len > 400)
    {
        RCLCPP_ERROR(rclcpp::get_logger("topic_register"),
                     "Abnormal string length detected for: %u. Max allowed: 2048. Skipping...", len);
        return true; // 报错退出，防止崩溃
    }
    return false;
}
bool get_topic_info(rosserial_msg::msg::TopicInfo &topic_info, Ros_Stream &msg)
{
    std::string topic_name;
    msg >> topic_info.topic_id;
    uint32_t name_len;
    msg >> name_len;
    if (excessive_testing(name_len))
    {
        return false;
    }
    topic_info.topic_name.resize(name_len);
    msg >> topic_info.topic_name;

    uint32_t type_len;
    msg >> type_len;
    if (excessive_testing(type_len))
    {
        return false;
    }
    topic_info.message_type.resize(type_len);
    msg >> topic_info.message_type;

    uint32_t md_len;
    msg >> md_len;
    if (excessive_testing(md_len))
    {
        return false;
    }
    topic_info.md5sum.resize(md_len);
    msg >> topic_info.md5sum;

    msg >> topic_info.buffer_size;
    return true;
}
}