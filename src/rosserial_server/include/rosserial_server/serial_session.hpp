#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <rclcpp/serialized_message.hpp>
#include <stdexcept>
#include "rosserial_server/async_read_buffer.hpp"
#include "rosserial_server/topic_handle.hpp"

namespace ros2_serial{

struct Serial_Config {
        Serial_Config(std::string port, int baud):port_(port), baud_(baud){}
        std::string port_;
        int baud_;
    };
    

//SerialDrive只作为普通的C++类进行实现，通过持有节点的弱指针来实现节点相关的函数调用，作为主体
class SerialDrive 
{
    public:
        SerialDrive (std::shared_ptr<Serial_Config> serial_config, rclcpp::Node* node, std::string node_name = "serial_drive"): 
        node_(node),serial_config_(serial_config) {
            //pub_ = publish_test(node_);
            //open_serial(serial_config_->port_, serial_config_->baud_);
            //start_read();
            sub_ = subscribe_test(node_);
        }
        //避免出现io_context停止后需要pull执行取消，但定时器已经销毁的情况
        ~SerialDrive (){
            std::cout<<"Destruct SerialDrive! "<<std::endl;
            if(!service_io_.stopped())
            {
                service_io_.stop();
            }
            if(serial_.is_open())
            {
                boost::system::error_code ec;
                serial_.cancel(ec);
                serial_.close(ec);
            }
        }

    private:
        std::shared_ptr<Serial_Config> serial_config_;
        boost::asio::io_context service_io_;
        boost::asio::serial_port serial_{service_io_};
        rclcpp::Node* node_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        int failed_connection_attempts_{0};
        int failed_close_attempts_{0};
        std::shared_ptr<AsyncReadBuffer<boost::asio::serial_port>> async_reader_;
        std::map<uint16_t, std::function<void(Ros_Stream&)>> callback_;
        std::shared_ptr<Serial_Publisher> pub_;
        std::shared_ptr<Serial_Subscriber>sub_;

    
    public:
        void run_io_contect()
        {
            size_t poll_handle = 0;
            while(service_io_.poll_one())
            {
                poll_handle++;
                if (poll_handle >50){
                    RCLCPP_DEBUG(node_->get_logger(), "The number of callbacks has reached the maximum limit.");
                    break;
                }
            }
            if (service_io_.stopped())
            {
                service_io_.restart();
            }
        }
        rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter>& params)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "success";
            for (const auto &param : params)
            {
                if (param.get_name() == "baud")
                {
                    int new_baud = param.as_int();
                    if (new_baud <= 0)
                    {
                        result.successful = false;
                        result.reason = "The baud value is wrong !";
                        return result;
                    }
                    RCLCPP_WARN (node_->get_logger(),"检测到修改波特率请求：%d -> %d",serial_config_->baud_, new_baud);
                //修改波特率相关实现
                if(reload_hardware(serial_config_->port_, new_baud))
                {
                    serial_config_->baud_ = new_baud;
                }
                else{
                    result.successful = false;
                    result.reason = "Setting buad failed";
                    reload_hardware(serial_config_->port_, serial_config_->baud_);
                    return result;
                }
                    RCLCPP_WARN(node_->get_logger(), "波特率修改成功！");
                }
    
                if (param.get_name() == "port"){
                    std::string new_port  = param.as_string();
                    if(new_port == ""){
                        result.successful = false;
                        result.reason = "The port name is null !";
                        return result;
                    }
                    RCLCPP_WARN(node_->get_logger(), "检测到修改端口名请求：%s -> %s", serial_config_->port_.c_str(), new_port.c_str());
                    if(!reload_hardware(new_port, serial_config_->baud_)){
                        result.successful = false;
                        result.reason = "Setting port failed";
                        reload_hardware(serial_config_->port_, serial_config_->baud_);
                        return result;
                    }
                    serial_config_->port_ = new_port;
                    RCLCPP_WARN(node_->get_logger(), "端口名修改成功！");
                }
            } 
            return result;
        }       
        
        void start_read()
        {
            RCLCPP_DEBUG(node_->get_logger(), "Start Reading");
            // async_reader_->read(10, std::bind(&SerialDrive::print, this, std::placeholders::_1));
            read_async_head();


        }
        //测试打印
        // void print(Ros_Stream& read_stream)
        // {
        //     char temp;
             
        //     for(size_t i =0; i < read_stream.get_length(); i++){ 

        //         read_stream >> temp;
        //         std::cout<<temp;
        //     }
        //     std::cout<<std::endl;

        //     start_read();
        // }

        void read_async_head(){
            async_reader_->read(1,std::bind(&SerialDrive::read_async_first, this, std::placeholders::_1));
        }
         void read_async_first(Ros_Stream& read_stream){
            uint8_t sync_flag;
            read_stream>>sync_flag;
            if(sync_flag == 0xFF){
                async_reader_->read(1,bind(&SerialDrive::read_async_second, this ,std::placeholders::_1));
            }
            else{
                //RCLCPP_DEBUG(node_->get_logger(), "No correct data packet header was found.");
                read_async_head();
            }

        }
        void read_async_second(Ros_Stream& read_stream){
            uint8_t  sync_version;
            read_stream >> sync_version;
    
            if(sync_version == 0xFE){
                async_reader_->read(5,bind(&SerialDrive::read_length, this ,std::placeholders::_1));
            }
            else {
                if(sync_version == 0xFF){
                    throw std::out_of_range("The Rosserial version is not applicable.");
                }
                else {
                    RCLCPP_DEBUG(node_->get_logger(), "No correct data packet header was found.");
                }
                read_async_head();
            }
        }
        void read_length(Ros_Stream& read_stream){
            uint16_t length, topic_id;
            uint8_t length_checksum;
            read_stream >> length >> length_checksum;
            if(datasum_for_check(length)+ length_checksum != 0xFF) {
                RCLCPP_WARN(node_->get_logger(), "The data length verification failed.");
                read_async_head();
            }
            else{
                 read_stream >> topic_id;
                 RCLCPP_DEBUG(node_-> get_logger(), "Rcceive message header with length %d and topic_id = %d", length, topic_id);
                 //读取数据主体
                 async_reader_->read(length +1, std::bind(&SerialDrive::read_body, this, std::placeholders::_1, topic_id));


            }

        }
        void read_body(Ros_Stream& read_stream, uint16_t topic_id){
            uint8_t data_checksum;
            //载荷数据读取
            Ros_Stream body_stream(read_stream.get_length()-1);
            read_stream >> body_stream;
            //数据校验位
            read_stream >> data_checksum;
            if(datasum_for_check(body_stream) + datasum_for_check(topic_id) + data_checksum != 0xFF ){
                RCLCPP_DEBUG(node_->get_logger(), "The main data verification failed.");
                read_async_head(); 
            }
            else{
                //调用topic_id对应的回调函数
                //待写一些判断逻辑
                //std::cout<< read_stream << std::endl;
                RCLCPP_DEBUG(node_-> get_logger(), "Rcceive message!");
                //pub_->handle(body_stream);
                //callback_[topic_id](body_stream);
                read_async_head();
            }

        }
        //计算长度的字节之和
        uint8_t datasum_for_check(uint16_t val){
            //将长度逐字节相加，利用返回值类型将高字节截断
            return (val >> 8) + val;
        }
        uint8_t datasum_for_check(Ros_Stream& body_stream){
            uint8_t sum{0};
            for(size_t i=0; i<body_stream.get_length(); i++){
                sum += body_stream.get_head()[i];
            }
            return sum;

        }
        //向下位机写入数据
        void write_to_serial(){


        }

    private:
        //打开串口
        bool open_serial(const std::string& port, const int& baud)
        {
            boost::system::error_code ec;
            if (this->serial_.is_open())
                this->serial_.close(ec);
            serial_.open(port, ec);
            if (ec)
            {
                failed_connection_attempts_++;
              if(failed_connection_attempts_== 1)
              {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to open port"<<port<<":"<<ec);
              }  
              else
              {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Unable to open port"<<port<<":"<<ec);
              }
              return false;
            }
            RCLCPP_DEBUG_STREAM(node_->get_logger(),"Open"<<":"<<port);
            failed_connection_attempts_ = 0;
            this->serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
            this->serial_.set_option(boost::asio::serial_port_base::character_size(8));
            this->serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            this->serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            this->serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            async_reader_ = std::make_shared <AsyncReadBuffer<boost::asio::serial_port>>(serial_, 512);
            return true;
        }   
        //用于修改串口参数后重启串口
        bool reload_hardware(std::string port, const int& baud)
        {
            boost::system::error_code ec;
            this->serial_.close(ec);
            if (ec)
            {
                failed_close_attempts_++;
              if(failed_close_attempts_== 1)
              {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to close port"<<serial_config_->port_<<":"<<ec);
              }  
              else
              {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Unable to close port"<<serial_config_->port_<<":"<<ec);
              }
              return false;
            }
            RCLCPP_DEBUG_STREAM(node_->get_logger(),"Open"<<":"<<serial_config_->port_);
            failed_close_attempts_ = 0;
            return (open_serial(port,baud));
        }

};
}
