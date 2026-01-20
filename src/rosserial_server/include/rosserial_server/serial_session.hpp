#pragma once
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include "rosserial_server/async_read_buffer.hpp"

namespace ros2_serial{

struct Serial_Config {
        Serial_Config(std::string port, int baud):port_(port), baud_(baud){}
        std::string port_;
        int baud_;
    };
    

//SerialDrive只作为普通的C++类进行实现，通过持有节点的弱指针来实现节点相关的函数调用
class SerialDrive 
{
    public:
        SerialDrive (std::shared_ptr<Serial_Config> serial_config, rclcpp::Node* node, std::string node_name = "serial_drive"): 
        node_(node),serial_config_(serial_config) {
            open_serial(serial_config_->port_, serial_config_->baud_);
            start_read();
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
            async_reader_->read(10, std::bind(&SerialDrive::print, this, std::placeholders::_1));


        }
        void print(const rclcpp::SerializedMessage& msg)
        {
            for(size_t i =0; i < msg.get_rcl_serialized_message().buffer_length; i++)
                std::cout<<msg.get_rcl_serialized_message().buffer[i];
            start_read();
        }


        private:
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
