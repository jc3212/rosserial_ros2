#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <stdexcept>
#include <vector>
#include "rosserial_server/protocol/serial_stream.hpp"


namespace ros2_serial
{
//计算长度的字节之和
uint8_t datasum_for_check(uint16_t val)
{
    //将长度逐字节相加，利用返回值类型将高字节截断
    return (val >> 8) + val;
}
//数据校验计算，对于流数据的重载
uint8_t datasum_for_check(Ros_Stream &body_stream)
{
    uint8_t sum{0};
    for (size_t i = 0; i < body_stream.get_length(); i++)
    {
        sum += body_stream.get_head()[i];
    }
    return sum;
}
//自定义用于存放反序列化成ros1数据的类
class Write_Buffer
{
  public:
    Write_Buffer(size_t len) : write_len_(0)
    {
        serial_msg_.resize(len);
    }
    size_t get_len() const
    {
        return serial_msg_.size();
    }
    size_t get_remain() const
    {
        return (get_len() - write_len_);
    }
    size_t get_write_len() const
    {
        return write_len_;
    }

    void ensure_size(size_t size)
    {
        if (size < serial_msg_.size())
        {
            return;
        }
        serial_msg_.resize(size);
    }
    uint8_t *get_wptr()
    {
        return serial_msg_.data() + write_len_;
    }
    const uint8_t *get_data() const
    {
        return serial_msg_.data();
    }
    const std::vector<uint8_t> &get_vector() const
    {
        return serial_msg_;
    }
    //真正读写的函数
    void buffer_write(const void *read_ptr, size_t n)
    {
        if (n == 0)
        {
            return;
        }
        if (!read_ptr)
        {
            throw std::invalid_argument("read_ptr is null");
        }
        if (get_remain() < n)
        {
            ensure_size(get_write_len() + n);
        }
        memcpy(get_wptr(), read_ptr, n);
        write_len_ += n;
    }
    //为平凡类设计的读写函数,针对数组写了对应的处理逻辑，通过连续空间的memcpy实现
    void read_and_mv_ptr(const void *read_ptr, const rosidl_typesupport_introspection_cpp::MessageMember &member,
                         size_t elem_size)
    {
        //对于存在的无上界数组（array_size == 0）进行判断处理
        if (member.is_array_ && (member.is_upper_bound_ || !member.array_size_))
        {
            throw std::runtime_error("sequence/vector not supported");
        }
        size_t count = member.is_array_ ? member.array_size_ : 1;
        buffer_write(read_ptr, count * elem_size);
    }

  private:
    size_t write_len_;
    std::vector<uint8_t> serial_msg_;
};
} // namespace
