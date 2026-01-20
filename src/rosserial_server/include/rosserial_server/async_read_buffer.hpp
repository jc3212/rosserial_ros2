#pragma once
#include <boost/asio/read.hpp>
#include <cstdint>
#include <cstdio>
#include <boost/asio.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace {
std::chrono::steady_clock::time_point last_stop_time_;

template <typename AsyncReadStream>
class AsyncReadBuffer
{
public:
    AsyncReadBuffer(AsyncReadStream& s, size_t capacity)
    :stream_(s),read_requested_bytes_(0)
    {
        reset();
        mem_.resize(capacity);
    }
    void read(size_t requested_bytes, std::function<void(const rclcpp::SerializedMessage&)>callback)
    {

        auto now = std::chrono::steady_clock::now();
        auto gap = now - last_stop_time_;
        RCLCPP_WARN(rclcpp::get_logger("gap_check"), "IO Gap detected: %ld us", gap);
        read_sussess_callback_ = callback;
        read_requested_bytes_ = requested_bytes;
        if(read_requested_bytes_ >mem_.size()||!read_requested_bytes_)
        {
            RCLCPP_WARN(rclcpp::get_logger("async_read"),"The requested bytes do not meet the requirements. requeset_bytes: %zu; max_size: %zu", read_requested_bytes_, mem_.size());
            return;
        }
        //先判断请求的字节数缓冲区能否满足
        if(read_requested_bytes_ > Already_bytes())
        {
            size_t diff_bytes = read_requested_bytes_ - Already_bytes();
            //不能满足，判断缓冲区到末尾空间大小是否足够
            if(diff_bytes > Tail_Remain_bytes())
            {
                if(read_index_ == write_index_){
                    reset();
                }
                //不够的话将缓冲区做移动操作  
                else{
                    memmove(&mem_[0],&mem_[read_index_],Already_bytes());   
                    //先移动write_index
                    write_index_ = Already_bytes();
                    read_index_ = 0;
                }

            }
        //在串口读取对应字节数据
        RCLCPP_DEBUG(rclcpp::get_logger("async_read"), "Writting Message from port.");
        // async_read读取会有默认的缓冲数据
        boost::asio::async_read(stream_, 
            boost::asio::buffer(&mem_[write_index_], Tail_Remain_bytes()),
            boost::asio::transfer_at_least(diff_bytes),
            std::bind(&AsyncReadBuffer::read_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
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
    AsyncReadStream& stream_;
    std::vector <uint8_t> mem_;
    size_t write_index_; 
    size_t read_index_;
    size_t read_requested_bytes_;
    size_t Already_bytes(){return write_index_- read_index_;};
    size_t Tail_Remain_bytes(){return mem_.size() - write_index_;};
    std::function<void(rclcpp::SerializedMessage serial_msg)> read_sussess_callback_;

    void read_callback(const boost::system::error_code& ec , const size_t& transfer_bytes){
        if(ec){
            read_requested_bytes_ = 0;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("async_read"),"Read operation failed with"<< ec);
            return;
        }
        write_index_ += transfer_bytes;
        if(read_requested_bytes_<= Already_bytes()){
            callSussessCallback();
        }
        else{
            stream_.async_read_some(
                boost::asio::buffer(&mem_[write_index_], Tail_Remain_bytes()),
                std::bind(&AsyncReadBuffer::read_callback, this, std::placeholders::_1, std::placeholders::_2)
            );       
        }
        if(read_index_ == write_index_){
            reset();
        }
    };
    void callSussessCallback(){
        // if(!rclcpp::ok()){
        //     return;
        // }
        last_stop_time_ = std::chrono::steady_clock::now(); 
        RCLCPP_DEBUG(rclcpp::get_logger("async_read"), "Reading Message from buffer.");
        rclcpp::SerializedMessage serial_msg(read_requested_bytes_);
        auto& rcl_msg = serial_msg.get_rcl_serialized_message();
        memcpy(rcl_msg.buffer, &mem_[read_index_],read_requested_bytes_);
        rcl_msg.buffer_length = read_requested_bytes_;
        read_index_ += read_requested_bytes_;
        boost::asio::post(stream_.get_executor(), std::bind(read_sussess_callback_, serial_msg));
    };



};

}