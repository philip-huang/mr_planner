#include "logger.h"


void Logger::setMethod(LogMethod newMethod) {
    std::lock_guard<std::mutex> lock(mtx);
    method = newMethod;
    if (method == LogMethod::FILE) {
        fileStream.open("log.txt", std::ofstream::out | std::ofstream::app);
        if (!fileStream.is_open()) {
            std::cerr << "Failed to open log file!" << std::endl;
            method = LogMethod::COUT; // Fallback to cout if file opening fails
        }
    } else if (fileStream.is_open()) {
        fileStream.close();
    }
}

void Logger::setLogLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(mtx);
    logLevel = level;
}

void Logger::log(const std::string& message, LogLevel level) {
    if (level < logLevel) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx);
    switch (method) {
        case LogMethod::ROS_INFO:
            // ROS_INFO("%s", message.c_str()); // Uncomment this if using ROS
            break;
        case LogMethod::COUT:
            std::cout << message << std::endl;
            break;
        case LogMethod::FILE:
            if (fileStream.is_open()) {
                fileStream << message << std::endl;
            }
            break;
        case LogMethod::NONE:
            // Do nothing
            break;
    }
}

void Logger::log(const RobotPose& pose, LogLevel level) {
    if (level < logLevel) {
        return;
    }
    assert(pose.joint_values.size() > 0); // Ensure the pose has joint values
    std::string message = "Robot " + pose.robot_name + " (" + std::to_string(pose.robot_id) + "), joint: ";
    for (const auto& value : pose.joint_values) {
        message += std::to_string(value) + " ";
    }
    log(message, level);
}

void Logger::log(const RobotTrajectory& traj, LogLevel level) {
    if (level < logLevel) {
        return;
    }
    std::string message = "Robot " + std::to_string(traj.robot_id) + ", trajectory: \n";
    assert(traj.trajectory.size() == traj.times.size());
    for (size_t i = 0; i < traj.trajectory.size(); ++i) {
        message += "Pose " + std::to_string(i) + " (t=" + std::to_string(traj.times[i]) + "): ";
        for (const auto& value : traj.trajectory[i].joint_values) {
            message += std::to_string(value) + " ";
        }
        message += "\n";
    }
    log(message, level);
}

Logger::Logger() : method(LogMethod::COUT), logLevel(LogLevel::INFO) {}

// Global functions for convenience
void setLogMethod(LogMethod method) {
    Logger::getInstance().setMethod(method);
}

void setLogLevel(LogLevel level) {
    Logger::getInstance().setLogLevel(level);
}

void log(const std::string& message, LogLevel level) {
    Logger::getInstance().log(message, level);
}

void log(const RobotPose& pose, LogLevel level) {
    Logger::getInstance().log(pose, level);
}

void log(const RobotTrajectory& traj, LogLevel level) {
    Logger::getInstance().log(traj, level);
}
