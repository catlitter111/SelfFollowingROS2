// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dlrobot_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice

#ifndef DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__BUILDER_HPP_
#define DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dlrobot_robot_msg/msg/detail/data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dlrobot_robot_msg
{

namespace msg
{

namespace builder
{

class Init_Data_z
{
public:
  explicit Init_Data_z(::dlrobot_robot_msg::msg::Data & msg)
  : msg_(msg)
  {}
  ::dlrobot_robot_msg::msg::Data z(::dlrobot_robot_msg::msg::Data::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dlrobot_robot_msg::msg::Data msg_;
};

class Init_Data_y
{
public:
  explicit Init_Data_y(::dlrobot_robot_msg::msg::Data & msg)
  : msg_(msg)
  {}
  Init_Data_z y(::dlrobot_robot_msg::msg::Data::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Data_z(msg_);
  }

private:
  ::dlrobot_robot_msg::msg::Data msg_;
};

class Init_Data_x
{
public:
  Init_Data_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Data_y x(::dlrobot_robot_msg::msg::Data::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Data_y(msg_);
  }

private:
  ::dlrobot_robot_msg::msg::Data msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dlrobot_robot_msg::msg::Data>()
{
  return dlrobot_robot_msg::msg::builder::Init_Data_x();
}

}  // namespace dlrobot_robot_msg

#endif  // DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__BUILDER_HPP_
