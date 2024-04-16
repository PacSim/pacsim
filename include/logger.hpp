#ifndef PACSCIMLOGGER_HPP
#define PACSCIMLOGGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>

class Logger
{
public:
    void logInfo(std::string in) { RCLCPP_INFO_STREAM(rclcpp::get_logger("pacsim_logger"), in); }

    void logWarning(std::string in) { RCLCPP_WARN_STREAM(rclcpp::get_logger("pacsim_logger"), in); }

    void logError(std::string in) { RCLCPP_ERROR_STREAM(rclcpp::get_logger("pacsim_logger"), in); }

    void logFatal(std::string in) { RCLCPP_FATAL_STREAM(rclcpp::get_logger("pacsim_logger"), in); }
};

#endif /* PACSCIMLOGGER_HPP */