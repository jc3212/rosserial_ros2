#pragma once

// Linux
#include <ament_index_cpp/get_package_prefix.hpp>
#include <dlfcn.h>
//中间层
#include <rcl/types.h>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "rosserial_server/protocol/message_serialization.hpp"
#include "rosserial_server/protocol/serial_stream.hpp"

#include <rosserial_msg/msg/detail/topic_info__struct.hpp>

namespace ros2_serial
{
//处理ros1和ros2之间的数据转换
class Stream_Deserialization
{
  public:
    Stream_Deserialization()
        : lib_handle_(nullptr){

          };
    ~Stream_Deserialization()
    {
        //关闭打开的句柄
        if (lib_handle_)
        {
            dlclose(lib_handle_);
        }
    }

    bool init(const rosserial_msg::msg::TopicInfo &topic_info)
    {
        std::string pkg_name, msg_name;
        size_t slash_pos = topic_info.message_type.find('/');
        if (slash_pos == std::string::npos)
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "Invalid type format: %s",
                         topic_info.message_type.c_str());
            return false;
        }
        pkg_name = topic_info.message_type.substr(0, slash_pos);
        msg_name = topic_info.message_type.substr(slash_pos + 1);
        //避免存在中间层的类型名干扰如 /std_msgs/msg/String
        size_t slash_second_pos = msg_name.find('/');
        if (slash_second_pos != msg_name.npos)
        {
            msg_name = msg_name.substr(slash_second_pos + 1);
        }
        if (!load_type_support(pkg_name, msg_name))
        {
            return false;
        }
        return true;
    }

    void publish(Ros_Stream &stream, rclcpp::GenericPublisher::SharedPtr pub)
    {
        if (!members_)
        {
            return;
        }
        //由于不知道具体消息的格式，只能手动进行new ClassNmae()的操作
        //调用构造函数
        //分配裸内存,此时仅有内存
        std::vector<uint8_t> raw_buffer(members_->size_of_);
        void *msg_obj = raw_buffer.data();
        //初始化raw_bufferk, placement new
        if (members_->init_function)
        {
            members_->init_function(msg_obj, rosidl_runtime_cpp::MessageInitialization::ALL);
        }
        //填充数据
        bool parse_success = deserialize_ros1_to_struct(stream, msg_obj, members_);
        if (!parse_success)
        {
            RCLCPP_WARN(rclcpp::get_logger("data_serialize"),
                        "Message deserialization failed, unable to publish correctly");
            if (members_->fini_function)
            {
                members_->fini_function(msg_obj);
            }
        }

        //将消息对象序列化成CDR数据
        //预分配CDR缓冲区
        rcutils_allocator_t allocator = rcutils_get_default_allocator();
        rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
        auto ret_init = rmw_serialized_message_init(&serialized_msg, 0u, &allocator); // 0u自动增长

        // 调用底层 RMW 序列化： C++ Object -> CDR Bytes
        // 这一步会自动处理 Padding 和 Byte Order
        if (ret_init == RMW_RET_OK)
        {
            auto ret_serialize = rmw_serialize(msg_obj, type_cdr_support_, &serialized_msg);
            if (ret_serialize == RMW_RET_OK)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "Message deserialization successful");
                rclcpp::SerializedMessage cpp_serialized_msg(serialized_msg);
                pub->publish(cpp_serialized_msg);
            }
            else
            {

                RCLCPP_WARN(rclcpp::get_logger("data_serialize"), "Message deserialization failed");
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "Failed to serialize message");
        }
        //析构对象
        if (members_->fini_function)
        {
            members_->fini_function(msg_obj);
        }
        //释放CDR缓冲区
        rmw_serialized_message_fini(&serialized_msg);
    }
    std::shared_ptr<Write_Buffer> subscribe(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg_ptr)
    {
        const rmw_serialized_message_t *rmw_serialized_msg = &serialized_msg_ptr->get_rcl_serialized_message();
        std::vector<uint8_t> raw_buffer(members_->size_of_);
        void *msg_obj = raw_buffer.data();
        serial_buffer_ = std::make_shared<Write_Buffer>(members_->size_of_);
        if (members_->init_function)
        {
            members_->init_function(msg_obj, rosidl_runtime_cpp::MessageInitialization::ALL);
        }
        //调用rmw结构反序列化
        rmw_ret_t ret_deserialize = rmw_deserialize(rmw_serialized_msg, type_cdr_support_, msg_obj);
        if (ret_deserialize == RMW_RET_OK)
        {
            //消息重新序列化成ros1格式
            if (serialized_struct_to_ros1(msg_obj, serial_buffer_, members_))
            {
                RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "rmw_deserialize from ros2 to ros1 sucess!");
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "rmw_deserialize from ros2 to ros1 failed!");
        }
        if (members_->fini_function)
        {
            members_->fini_function(msg_obj);
        }
        return serial_buffer_;
    }

  private:
    //保存动态库句柄
    void *lib_handle_;
    //反射信息句柄
    const rosidl_typesupport_introspection_cpp::MessageMembers *members_;
    //类型支持句柄
    const rosidl_message_type_support_t *type_support_;
    const rosidl_message_type_support_t *type_cdr_support_;
    std::shared_ptr<Write_Buffer> serial_buffer_;
    bool deserialize_ros1_to_struct(Ros_Stream &stream, void *msg_obj,
                                    const rosidl_typesupport_introspection_cpp::MessageMembers *members)
    {
        //将void*指针转换成uint_8指针
        uint8_t *base_ptr = static_cast<uint8_t *>(msg_obj);

        for (size_t i = 0; i < members->member_count_; i++)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "DEBUG: Enter deserialize. Msg: %s, Members Addr: %p",
                        members->message_name_, (void *)members);

            const auto &member = members->members_[i];
            uint8_t *mem_ptr = base_ptr + members->members_[i].offset_;
            RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "DEBUG: Parsing member: %s, type: %d", member.name_,
                        member.type_id_);
            if (member.is_array_ && !member.is_upper_bound_ && member.array_size_ > 0 &&
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE)
            {
                //处理double数组
                double *arry_ptr = static_cast<double *>(reinterpret_cast<void *>(mem_ptr));
                for (size_t k = 0; k < member.array_size_; k++)
                {
                    stream >> arry_ptr[k];
                }
            }
            else if (member.is_array_)
            {
                RCLCPP_WARN(rclcpp::get_logger("data_serialize"), "Arrays not supported yet, skipping field %s",
                            member.name_);
                return false; // 暂时直接报错，防止错位
            }
            else
            {
                switch (member.type_id_)
                {
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
                    bool val;
                    stream >> val;
                    *static_cast<bool *>(reinterpret_cast<void *>(mem_ptr)) = val;
                    break;
                }
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE: {
                    uint8_t val;
                    stream >> val;
                    *static_cast<uint8_t *>(reinterpret_cast<void *>(mem_ptr)) = val;
                    break;
                }
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
                    int32_t val;
                    stream >> val;
                    *static_cast<uint32_t *>(reinterpret_cast<void *>(mem_ptr)) = val;
                    break;
                }
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
                    int32_t val;
                    stream >> val;
                    *static_cast<uint32_t *>(reinterpret_cast<void *>(mem_ptr)) = val;
                    break;
                }
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
                    float val;
                    stream >> val;
                    *static_cast<float *>(reinterpret_cast<void *>(mem_ptr)) = val;
                    break;
                }
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
                    double val;
                    stream >> val;
                    *static_cast<double *>(reinterpret_cast<void *>(mem_ptr)) = val;
                    break;
                }
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
                    std::string *str_ptr = static_cast<std::string *>(reinterpret_cast<void *>(mem_ptr));
                    uint32_t len;
                    stream >> len;
                    if (len > 400)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("data_serialize"),
                                     "Abnormal string length detected for %s: %u. Max allowed: 2048. Skipping...",
                                     member.name_, len);
                        return false; // 报错退出，防止崩溃
                    }
                    str_ptr->resize(len);
                    stream >> *str_ptr;
                    break;
                }
                // ROS消息嵌套的消息类型
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
                    const auto *sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
                        member.members_->data);
                    //去掉ros1消息中可能存在的seq字段
                    if (strcmp(member.name_, "header") == 0)
                    {
                        bool is_std_header = false;
                        for (size_t h = 0; h < sub_members->member_count_; h++)
                        {
                            //判断条件，
                            //在header中，且该header是std/msg定义的而不是自定义的，通过检查是否存在stamp字段判断
                            if (!strcmp(sub_members->members_[h].name_, "stamp"))
                            {
                                is_std_header = true;
                                break;
                            }
                        }
                        if (is_std_header)
                        {
                            uint32_t seq;
                            stream >> seq;
                        }
                    }
                    RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "DEBUG: Sub-message name: %s, Addr: %p",
                                sub_members->message_name_, (void *)sub_members);
                    if (!deserialize_ros1_to_struct(stream, static_cast<void *>(mem_ptr), sub_members))
                    {
                        return false;
                    }
                    break;
                }
                default: {
                    RCLCPP_WARN(rclcpp::get_logger("data_serialize"),
                                "Undefined type processing for member %s, type_id: %d", member.name_, member.type_id_);
                    return false;
                }
                }
            }
        }
        return true;
    }

    //将反序列化的ros2消息转换成ros1要求的紧密型数据
    bool serialized_struct_to_ros1(void *msg_obj, std::shared_ptr<Write_Buffer> serial_buffer,
                                   const rosidl_typesupport_introspection_cpp::MessageMembers *members)
    {
        Ros_Stream stream(static_cast<uint8_t *>(msg_obj), members->size_of_);

        for (size_t i = 0; i < members->member_count_; i++)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"),
                        "DEBUG: Serialize the structure into padding format data. Msg: %s, Members Addr: %p",
                        members->message_name_, (void *)members_);
            const auto &member = members->members_[i];

            uint8_t *msg_ptr = static_cast<uint8_t *>(msg_obj) + member.offset_;
            RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "DEBUG: Parsing member: %s, type: %d", member.name_,
                        member.type_id_);

            switch (member.type_id_)
            {
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
                serial_buffer->read_and_mv_ptr(msg_ptr, member, sizeof(bool));
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE: {
                serial_buffer->read_and_mv_ptr(msg_ptr, member, sizeof(uint8_t));
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
                serial_buffer->read_and_mv_ptr(msg_ptr, member, sizeof(int32_t));
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
                serial_buffer->read_and_mv_ptr(msg_ptr, member, sizeof(uint32_t));

                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
                serial_buffer->read_and_mv_ptr(msg_ptr, member, sizeof(float));

                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
                serial_buffer->read_and_mv_ptr(msg_ptr, member, sizeof(double));
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
                if (member.is_array_ && (member.is_upper_bound_ || !member.array_size_))
                {
                    throw std::runtime_error("sequence/string not supported");
                }
                size_t array_size = member.is_array_ ? member.array_size_ : 1;
                const std::string *str_ptr = reinterpret_cast<const std::string *>(msg_ptr);
                for (size_t k = 0; k < array_size; k++)
                {
                    const std::string &msg_string = str_ptr[k];
                    uint32_t len = msg_string.size();
                    // rosserail中格式对于string先写入长度
                    //不涉及到数组直接写入
                    serial_buffer->buffer_write(&len, sizeof(uint32_t));
                    //前面已经针对数组做了处理，直接写入string字符串
                    serial_buffer->buffer_write(msg_string.data(), msg_string.size());
                }
                break;
            }
            // ROS消息嵌套的消息类型
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {

                const auto *sub_members =
                    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
                //去掉ros1消息中可能存在的seq字段
                if (!member.array_size_ && strcmp(member.name_, "header") == 0)
                {
                    bool is_std_header = false;
                    for (size_t h = 0; h < sub_members->member_count_; h++)
                    {
                        //判断条件， 在header中，且该header是std/msg定义的而不是自定义的，通过检查是否存在stamp字段判断
                        if (!strcmp(sub_members->members_[h].name_, "stamp"))
                        {
                            is_std_header = true;
                            break;
                        }
                    }
                    if (is_std_header)
                    {
                        uint32_t seq = 0;
                        serial_buffer->buffer_write(&seq, sizeof(uint32_t));
                    }
                }
                size_t array_size = member.is_array_ ? member.array_size_ : 1;
                size_t sub_msg_len = sub_members->size_of_;
                for (size_t k = 0; k < array_size; k++)
                {
                    void *sub_msg_ptr = static_cast<uint8_t *>(msg_ptr) + k * sub_msg_len;
                    RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "DEBUG: Sub-message name: %s, Addr: %p",
                                sub_members->message_name_, (void *)sub_members);
                    if (!serialized_struct_to_ros1(static_cast<void *>(sub_msg_ptr), serial_buffer, sub_members))
                    {
                        return false;
                    }
                }

                break;
            }
            default: {
                RCLCPP_WARN(rclcpp::get_logger("data_serialize"),
                            "Undefined type processing for member %s, type_id: %d", member.name_, member.type_id_);
                return false;
            }
            }
        }
        return true;
    }

    bool load_type_support(const std::string &pkg_name, const std::string &msg_name)
    {
        std::string prefix;
        try
        {
            prefix = ament_index_cpp::get_package_prefix(pkg_name);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "Cannot find package!");
            return false;
        }
        std::string lib_name = "lib" + pkg_name + "__rosidl_typesupport_introspection_cpp.so";
        std::string lib_path = prefix + "/lib/" + lib_name;
        if (access(lib_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "Library file does NOT exist at parsed path: %s",
                         lib_path.c_str());
            return false; // 提前拦截，防止 dlopen 去加载错误的系统库或直接崩溃
        }
        RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "Attempting to load library: %s", lib_name.c_str());
        // 作用：将共享库文件加载到当前进程的地址空间
        // RTLD_LAZY: 懒加载（用到符号时才解析），提高加载速度
        // RTLD_GLOBAL: 让库里的符号对后续加载的库可见
        lib_handle_ = dlopen(lib_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
        if (!lib_handle_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "Cannot load library!");
            return false;
        }
        else
        {
            RCLCPP_DEBUG(rclcpp::get_logger("data_serialize"), "Loading library sucess!");
        }
        //查找符号
        std::string symbol =
            "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + pkg_name + "__msg__" + msg_name;
        void *sym_ptr = dlsym(lib_handle_, symbol.c_str());
        if (!sym_ptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "dlsym failed: %s", dlerror());
        }
        //获取TypeSupport句柄
        //直接找到能够返回话题type_support的动态链接库中的函数地址
        //将该地址转为FunPtr类型的函数入口
        using FunPtr = const rosidl_message_type_support_t *(*)();
        auto func = reinterpret_cast<FunPtr>(sym_ptr);
        type_support_ = func();

        //将type_support_转换成C++可用的结构体类型，具体来说，type_support_只是一个通用的句柄
        //编译器无法知道这个指针指向的是什么
        //这里相当于向编译器说明以MessageMember的格式来进行解析
        members_ = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support_->data);

        //加载将消息对象序列化成CDR时需要的中间层的通用库
        std::string lib_name_cpp = "lib" + pkg_name + "__rosidl_typesupport_cpp.so";
        void *lib_handle_cpp = dlopen(lib_name_cpp.c_str(), RTLD_LAZY);
        if (!lib_handle_cpp)
        {
            RCLCPP_WARN(rclcpp::get_logger("data_serialize"), "Failed to load typesconstupport_cpp: %s", dlerror());
            return false;
        }
        std::string func_name_cpp =
            "rosidl_typesupport_cpp__get_message_type_support_handle__" + pkg_name + "__msg__" + msg_name;
        auto func_cpp = (const rosidl_message_type_support_t *(*)())dlsym(lib_handle_cpp, func_name_cpp.c_str());
        if (!func_cpp)
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_serialize"), "Serial dlsym failed: %s", dlerror());
            return false;
        }
        type_cdr_support_ = func_cpp();

        return true;
    }
};
} // namespace ros2_serial