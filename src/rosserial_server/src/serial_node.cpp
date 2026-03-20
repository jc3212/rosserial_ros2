
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rosserial_server/serial_session.hpp"
#include <chrono>

namespace ros2_serial{
using namespace std::chrono_literals;
class SerialNode :public rclcpp::Node {
public: 
    //传入SerialNode节点指针
    SerialNode(std::string port = "/dev/ttyUSB_stm32", int baud = 115200,std::string node_name="serial_node") : Node (node_name){
        serial_config_ = std::make_shared<Serial_Config>(port = this->declare_parameter("port", port), baud = this->declare_parameter("baud", baud));
        serial_drive_ = std::make_unique<SerialDrive>(serial_config_,this);
        //动态参数配置
        //bind接受的是对象的裸指针，因此需要对智能指针进行转换
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&SerialDrive::parameter_callback, serial_drive_.get(), std::placeholders::_1));
        //采用定时器来定时处理boost的回调
        timer_ = this->create_wall_timer(10ms, [this](){
            if (!serial_drive_) {
                RCLCPP_ERROR(this->get_logger(), "FATAL: serial_drive_ is nullptr!");
                return;
            }
            RCLCPP_DEBUG(this->get_logger(), "Polling IO context...");
            serial_drive_->run_io_contect();});
    }
    SerialNode(int test_flag,std::string node_name):Node(node_name){
        serial_drive_ = std::make_unique<SerialDrive>(serial_config_,this);
    }

private:
    std::shared_ptr<Serial_Config> serial_config_;
    std::unique_ptr<SerialDrive> serial_drive_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rclcpp::TimerBase::SharedPtr timer_;

};

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<ros2_serial::SerialNode>();
    auto node = std::make_shared<ros2_serial::SerialNode>(1,"test_node");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

