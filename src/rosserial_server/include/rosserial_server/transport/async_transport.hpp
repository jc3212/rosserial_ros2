#pragma once
#include "rosserial_server/protocol/serial_stream.hpp"
#include "rosserial_server/protocol/message_serialization.hpp"
#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <ostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace ros2_serial
{
std::chrono::steady_clock::time_point last_stop_time_;

template <typename AsyncReadStream> class AsyncReadBuffer
{
  public:
    AsyncReadBuffer(AsyncReadStream &s, size_t capacity) : stream_(s), read_requested_bytes_(0)
    {
        reset();
        mem_.resize(capacity);
    }
    void read(size_t requested_bytes, std::function<void(Ros_Stream &)> callback)
    {

        //用于测试读取时间，判断是否在读取时存在数据积压不返回问题
        auto now = std::chrono::steady_clock::now();
        auto gap = now - last_stop_time_;
        // RCLCPP_WARN(rclcpp::get_logger("gap_check"), "IO Gap detected: %ld us", gap);
        //传入的处理读取到数据的回调函数
        read_sussess_callback_ = callback;
        read_requested_bytes_ = requested_bytes;
        //请求数据是否超过缓冲区大小
        if (read_requested_bytes_ > mem_.size() || !read_requested_bytes_)
        {
            RCLCPP_WARN(rclcpp::get_logger("async_read"),
                        "The requested bytes do not meet the requirements. requeset_bytes: %zu; max_size: %zu",
                        read_requested_bytes_, mem_.size());
            return;
        }
        //先判断请求的字节数缓冲区能否满足
        if (read_requested_bytes_ > Already_bytes())
        {
            size_t diff_bytes = read_requested_bytes_ - Already_bytes();
            //不能满足，判断缓冲区到末尾空间大小是否足够
            if (diff_bytes > Tail_Remain_bytes())
            {
                if (read_index_ == write_index_)
                {
                    reset();
                }
                //不够的话将缓冲区做移动操作
                else
                {
                    memmove(&mem_[0], &mem_[read_index_], Already_bytes());
                    //先移动write_index
                    write_index_ = Already_bytes();
                    read_index_ = 0;
                }
            }
            //在串口读取对应字节数据
            RCLCPP_DEBUG(rclcpp::get_logger("async_read"), "Writting Message from port.");
            // async_read读取会有默认的缓冲数据
            boost::asio::async_read(
                stream_, boost::asio::buffer(&mem_[write_index_], Tail_Remain_bytes()),
                boost::asio::transfer_at_least(diff_bytes),
                std::bind(&AsyncReadBuffer::read_callback, this, std::placeholders::_1, std::placeholders::_2));
            // stream_.async_read_some(
            //     boost::asio::buffer(&mem_[write_index_], Tail_Remain_bytes()),
            //     std::bind(&AsyncReadBuffer::read_callback, this, std::placeholders::_1, std::placeholders::_2)
            // );
        }
        else
        {
            callSussessCallback();
        }
    }
    void reset()
    {
        read_index_ = 0;
        write_index_ = 0;
    }

  private:
    // stream_是传入的socket,串口的读取都要依赖这个句柄完成
    AsyncReadStream &stream_;
    std::vector<uint8_t> mem_;
    size_t write_index_;
    size_t read_index_;
    size_t read_requested_bytes_;
    size_t Already_bytes()
    {
        return write_index_ - read_index_;
    };
    size_t Tail_Remain_bytes()
    {
        return mem_.size() - write_index_;
    };
    //   std::function<void(rclcpp::SerializedMessage serial_msg)> read_sussess_callback_;
    std::function<void(Ros_Stream &)> read_sussess_callback_;

    void read_callback(const boost::system::error_code &ec, const size_t &transfer_bytes)
    {
        if (ec)
        {
            read_requested_bytes_ = 0;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("async_read"), "Read operation failed with" << ec.message());
            return;
        }
        write_index_ += transfer_bytes;
        //判断已获取到的字节数是否足够
        if (read_requested_bytes_ <= Already_bytes())
        {
            callSussessCallback();
        }
        //使用read_some进行读取，一读到数据就返回
        else
        {
            stream_.async_read_some(
                boost::asio::buffer(&mem_[write_index_], Tail_Remain_bytes()),
                std::bind(&AsyncReadBuffer::read_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
    };
    void callSussessCallback()
    {
        // if(!rclcpp::ok()){
        //     return;
        // }
        // last_stop_time_ = std::chrono::steady_clock::now();
        // RCLCPP_DEBUG(rclcpp::get_logger("async_read"), "Reading Message from buffer.");
        // rclcpp::SerializedMessage serial_msg(read_requested_bytes_);
        // auto& rcl_msg = serial_msg.get_rcl_serialized_message();
        // memcpy(rcl_msg.buffer, &mem_[read_index_],read_requested_bytes_);
        // rcl_msg.buffer_length = read_requested_bytes_;
        // read_index_ += read_requested_bytes_;

        //使用rosserial式的字节流的形式
        Ros_Stream read_stream_(&mem_[read_index_], read_requested_bytes_);

        read_index_ += read_requested_bytes_;
        //判断已将缓冲区读完

        //将处理数据的回调函数放入回调函数组
        boost::asio::post(stream_.get_executor(), std::bind(read_sussess_callback_, read_stream_));
        // Asio 的队列里，排队的是【回调函数】，而不是【发送动作】
        //而读取命令只在回调函数中调用，在这个回调执行之前不会读取新的数据，也就不存在覆盖缓冲区的问题，因此这里直接用缓冲区指针没有问题
        if (read_index_ == write_index_)
        {
            reset();
        }
    };
};

//用于向串口发送数据
template <typename AsyncWriteStream> class AsyncWriteBuffer
{
  public:
    AsyncWriteBuffer(AsyncWriteStream &s) : stream_(s)
    {
    }
    //向下位机写入数据
    void write_to_serial(const std::shared_ptr<Write_Buffer>serial_msg, const uint16_t topic_id)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("write_to_socket"),"The length of the pure data payload is %d", serial_msg->get_write_len());
        std::shared_ptr<Write_Buffer> write_buf = packaging_msg(serial_msg, topic_id);
        std::vector<uint8_t> payload = write_buf->get_vector();
        std::lock_guard<std::mutex> lock(write_mtx_);
        //以队列是否为空作为是否正在发送的天然依据
        bool need_write = send_bufs_.empty();
        send_bufs_.push_back(std::move(payload));
        // send_bufs_不为空则说明有async_write持续写入
        if (need_write)
        {
            do_write();
        }
    }

  private:
    std::deque<std::vector<uint8_t>> send_bufs_;
    //互斥锁用于保护缓冲区
    std::mutex write_mtx_;
    AsyncWriteStream &stream_;
    std::shared_ptr<Write_Buffer> packaging_msg(const std::shared_ptr<Write_Buffer> &serial_msg,
                                                const uint16_t topic_id) const
    {
        uint8_t overhead_bytes = 8;
        uint16_t data_len = serial_msg->get_write_len();
        uint16_t length = overhead_bytes + data_len;

        std::shared_ptr<Write_Buffer> write_buf = std::make_shared<Write_Buffer>(length);

        //写校验头，小端序
        //在x86平台上如果直接用 uint16_t flag = 0xfffe ,在内存中以0xfe,0xff的顺序存放，导致错误
        uint8_t sync_flags[2] = {0xff, 0xfe};
        write_buf->buffer_write(sync_flags, sizeof(sync_flags));
        //写长度
        write_buf->buffer_write(&data_len, sizeof(uint16_t));
        //长度校验和
        uint8_t len_check = 0xFF - datasum_for_check(data_len);
        write_buf->buffer_write(&len_check, sizeof(uint8_t));
        // topic_id
        write_buf->buffer_write(&topic_id, sizeof(uint16_t));
        //实际数据
        write_buf->buffer_write(serial_msg->get_data(), data_len);
        //消息校验和
        Ros_Stream msg_stream(serial_msg->get_data(), data_len);
        uint8_t msg_check = 0xFF - (datasum_for_check(msg_stream) + datasum_for_check(topic_id));
        write_buf->buffer_write(&msg_check, sizeof(uint8_t));
        RCLCPP_DEBUG(rclcpp::get_logger("write_to_socket"),"The complete length of the data packet is %d", write_buf->get_write_len());
        return write_buf;
    }

    void do_write()
    {
        RCLCPP_INFO(rclcpp::get_logger("write_to_socket"),"The complete length of the payload is %d", send_bufs_.front().size());
        // async_write接受的地址参数要使用boost::asio::buffer()进行包装
        boost::asio::async_write(stream_, boost::asio::buffer(send_bufs_.front()),
                                 [this](boost::system::error_code error, const size_t bytes_transferred) {
                                     write_ec_callback(error, bytes_transferred);
                                 });
    }
    void write_ec_callback(boost::system::error_code error, const size_t bytes_transferred)
    {
        std::lock_guard<std::mutex> lock(write_mtx_);
        if (error)
        {
            if (error == boost::asio::error::operation_aborted)
            {
                RCLCPP_WARN(rclcpp::get_logger("write_to_socket"), "Serial write aborted (normal shutdown or cancel).");
            }
            else
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("write_to_socket"),
                                   "Unknown error returned during write operation: " << error.message());
            }
            //对应的处理逻辑
            send_bufs_.clear();
            return;
        }

        send_bufs_.pop_front();
        if (!send_bufs_.empty())
        {
            do_write();
        }
    }
};

} // namespace ros2_serial