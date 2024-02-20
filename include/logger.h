#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <instance.h>
#include <SingleAgentPlanner.h>

// Define logging methods
enum class LogMethod {
    ROS_INFO,
    COUT,
    FILE,
    NONE
};

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    HLINFO = 2,
    WARN = 3,
    ERROR = 4,
};

class Logger {
public:
    // Delete copy constructor and assignment operator to prevent copying
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Access the singleton instance of the Logger
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    };

    void setMethod(LogMethod newMethod);

    void setLogLevel(LogLevel level);

    void log(const std::string& message, LogLevel level);
    void log(const RobotPose& pose, LogLevel level);
    void log(const RobotTrajectory& traj, LogLevel level);

private:
    LogMethod method;

    LogLevel logLevel;

    std::ofstream fileStream;
    std::mutex mtx; // For thread safety

    // Private constructor for singleton
    Logger();
};

// Global functions for convenience
void setLogMethod(LogMethod method);

void setLogLevel(LogLevel level);

void log(const std::string& message, LogLevel level = LogLevel::INFO);

void log(const RobotPose& pose, LogLevel level = LogLevel::INFO);

void log(const RobotTrajectory& traj, LogLevel level = LogLevel::INFO);
