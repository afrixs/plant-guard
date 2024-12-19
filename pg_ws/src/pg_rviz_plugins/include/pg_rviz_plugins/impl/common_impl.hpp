//
// Created by matej on 11/12/24.
//

#include "rclcpp/serialization.hpp"

using namespace std::chrono_literals;

namespace pg_rviz_plugins {

template<typename MsgT>
typename MsgT::Response::SharedPtr callService(typename rclcpp::Client<MsgT>::SharedPtr client, const typename MsgT::Request::SharedPtr& request, rclcpp::executors::SingleThreadedExecutor &executor)
{
  bool connected = client->wait_for_service(800ms);
  if (!connected)
    return nullptr;
  auto result = client->async_send_request(request);
  typename MsgT::Response::SharedPtr response;
  if (executor.spin_until_future_complete(result, 800ms) == rclcpp::FutureReturnCode::SUCCESS &&
      (response = result.get())) {
    return response;
  }
  return nullptr;
}

template<typename T>
void deserialize_message(const std::vector<uint8_t>& data, T& msg) {
  rcl_serialized_message_t rcl_msg = rmw_get_zero_initialized_serialized_message();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rmw_serialized_message_init(&rcl_msg, data.size(), &allocator);
  rcl_msg.buffer_length = data.size();
  memcpy(rcl_msg.buffer, data.data(), data.size());
  const rclcpp::SerializedMessage serialized_message(rcl_msg);
  rclcpp::Serialization<T> serializer;
  serializer.deserialize_message(&serialized_message, &msg);
}

template<typename T>
void serialize_message(const T& msg, std::vector<uint8_t>& data) {
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(&msg, &serialized_msg);
  auto rcl_msg = serialized_msg.get_rcl_serialized_message();
  data.resize(rcl_msg.buffer_length);
  memcpy(data.data(), rcl_msg.buffer, rcl_msg.buffer_length);
}


}
