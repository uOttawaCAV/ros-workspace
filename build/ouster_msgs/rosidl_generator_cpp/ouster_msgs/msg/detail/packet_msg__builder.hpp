// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ouster_msgs:msg/PacketMsg.idl
// generated code does not contain a copyright notice

#ifndef OUSTER_MSGS__MSG__DETAIL__PACKET_MSG__BUILDER_HPP_
#define OUSTER_MSGS__MSG__DETAIL__PACKET_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ouster_msgs/msg/detail/packet_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ouster_msgs
{

namespace msg
{

namespace builder
{

class Init_PacketMsg_buf
{
public:
  Init_PacketMsg_buf()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ouster_msgs::msg::PacketMsg buf(::ouster_msgs::msg::PacketMsg::_buf_type arg)
  {
    msg_.buf = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ouster_msgs::msg::PacketMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ouster_msgs::msg::PacketMsg>()
{
  return ouster_msgs::msg::builder::Init_PacketMsg_buf();
}

}  // namespace ouster_msgs

#endif  // OUSTER_MSGS__MSG__DETAIL__PACKET_MSG__BUILDER_HPP_
