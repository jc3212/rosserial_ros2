#pragma once
#include <boost/asio/read.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <boost/asio.hpp>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::chrono::steady_clock::time_point last_stop_time_;

//从紧密型数据中读取各个类型的流式操作符
class Ros_Stream{
    public:   
        Ros_Stream (const uint8_t* buffer, const size_t length): head_(buffer),length_(length),cur_(buffer),end_(buffer+length){
        };
        Ros_Stream (const size_t length): head_(nullptr),length_(length),cur_(nullptr),end_(nullptr){
        };
        template<typename T>
        Ros_Stream& operator >> (T& value){
            if(head_ == nullptr){
                throw std::out_of_range("The accessed Stream has not been initialized!");
            }
            //获取要读取的类型长度
            size_t value_length = sizeof(value);
            RCLCPP_DEBUG(rclcpp::get_logger("async_read"),"Request bytes : %d , Remain bytes : %d", value_length, this->get_length());
            

            //判断缓冲区中剩余长度是否足够
            if (cur_+value_length <= end_){
                //小端序
                std::memcpy(&value, cur_, value_length);
                cur_ += value_length;
            }
            else{
                throw std::out_of_range("Serialization data access out of bounds!");

                }

            return *this;

        }
        Ros_Stream& operator >>(std::string& val_string){
            if(head_ == nullptr){
                throw std::out_of_range("The accessed Stream has not been initialized!");
            }
            size_t len = val_string.size();
            if(!len){
                throw std::out_of_range("The length of the string to be written is undefined.");
            }
            if (get_length()< len){
                throw std::out_of_range("Serialization data access out of bounds!");
            }
            else{
                memcpy(&val_string[0], cur_, len);
                cur_ += len;
            }
            return *this;
        }
        Ros_Stream& operator >>(Ros_Stream& write_stream){

            size_t request_length = write_stream.get_length();

            if(head_ == nullptr){
                throw std::out_of_range("The accessed Stream has not been initialized!");
            }
            //获取要读取的类型长度
            RCLCPP_DEBUG(rclcpp::get_logger("async_read"),"Request bytes : %d , Remain bytes : %d", request_length, this->get_length());

            //判断缓冲区中剩余长度是否足够
            if (cur_+request_length <= end_){
                //小端序
                write_stream.head_ = cur_;
                write_stream.length_ = request_length;
                write_stream.cur_ = cur_;
                write_stream.end_ = cur_ + request_length;
                cur_ += request_length;
            }
            else{
                throw std::out_of_range("Serialization data access out of bounds!");

                }

            return *this;
        }
        //从消息结构体中读取string,此时的string不再是只有字符串
        bool read_str_from_struct(std::string& val_string){
            size_t size_string = sizeof(std::string);
            if(head_ == nullptr){
                throw std::out_of_range("The accessed Stream has not been initialized!");
            }
            if (get_length()< size_string){
                throw std::out_of_range("Attempted to read std::string object beyond ROS 2 struct bounds!");
                return false;
            }
            else{
                const std::string* str_ptr = reinterpret_cast<const std::string*>(cur_);
                val_string = *str_ptr;
                
                cur_ += size_string;
            }
            return true;
        }
        size_t get_length()const {
            if(head_== nullptr&&end_==nullptr){
                RCLCPP_DEBUG(rclcpp::get_logger("async_read"),"The pointer has not been initialized yet! The returned value is the initialized length.");
                get_total_length();
            }
            else{
                return static_cast<int>(end_ - cur_);

            }
        }
        size_t get_total_length()const{
            return length_;
        }
        const uint8_t* get_head()const{
            return head_; 
        }
        const uint8_t* get_cur()const{
            return cur_;
        }


    private:

        const uint8_t* cur_;
        const uint8_t* head_;
        const uint8_t* end_;
        size_t length_;

};
//用于打印测试
std::ostream& operator << (std::ostream& os, const Ros_Stream& stream) {
    const size_t length = stream.get_total_length();
    const uint8_t* data = stream.get_head();
    
    //仅备份当前流的标志位和填充字符，极其轻量
    std::ios_base::fmtflags old_flags = os.flags();
    char old_fill = os.fill();

    // 设置流格式为十六进制，并用 0 填充
    os << std::hex << std::setfill('0');

    os << "[ Length: " << std::dec << length << " bytes ]\n[ "; // 先用十进制打印总长度，再开始数据

    // 切回十六进制
    os << std::hex;
    for (size_t i = 0; i < length; i++) {
        //每次输出完一个字节后，加上一个空格！
        os << "0x" << std::setw(2) << static_cast<int>(data[i]) << " ";
        
        // 每 16 个字节换一次行
        if ((i + 1) % 16 == 0 && (i + 1) != length) {
            os << "\n  ";
        }
    }
    os << "]\n";

    //恢复流的原始状态
    os.flags(old_flags);
    os.fill(old_fill);

    return os;
}



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
    void read(size_t requested_bytes, std::function<void(Ros_Stream&)>callback)
    {

        auto now = std::chrono::steady_clock::now();
        auto gap = now - last_stop_time_;
        //RCLCPP_WARN(rclcpp::get_logger("gap_check"), "IO Gap detected: %ld us", gap);
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

    //stream_是传入的socket,串口的读取都要依赖这个句柄完成
    AsyncReadStream& stream_;
    std::vector <uint8_t> mem_;
    size_t write_index_; 
    size_t read_index_;
    size_t read_requested_bytes_;
    size_t Already_bytes(){return write_index_- read_index_;};
    size_t Tail_Remain_bytes(){return mem_.size() - write_index_;};
//   std::function<void(rclcpp::SerializedMessage serial_msg)> read_sussess_callback_;
    std::function<void(Ros_Stream&)> read_sussess_callback_;

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

        boost::asio::post(stream_.get_executor(), std::bind(read_sussess_callback_, read_stream_));
    };



};

}