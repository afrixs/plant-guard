//
// Created by matej on 11/12/24.
//

#ifndef PLANT_GUARD_PG_RVIZ_PLUGINS_COMMON_HPP
#define PLANT_GUARD_PG_RVIZ_PLUGINS_COMMON_HPP

#include <rclcpp/rclcpp.hpp>

namespace pg_rviz_plugins {

extern const std::string ALLOWED_NAMING_PATTERN;
extern const std::string CRONTAB_SCHEDULE_PATTERN;

template<class MsgT>
typename MsgT::Response::SharedPtr callService(typename rclcpp::Client<MsgT>::SharedPtr client, const typename MsgT::Request::SharedPtr& request, rclcpp::executors::SingleThreadedExecutor &executor);

template<typename T>
void deserialize_message(const std::vector<uint8_t>& data, T& msg);

template<typename T>
void serialize_message(const T& msg, std::vector<uint8_t>& data);

bool parse_double(const std::string &in, double& res, double default_value = 0.0);

}

#include "impl/common_impl.hpp"

#endif //PLANT_GUARD_COMMON_HPP
