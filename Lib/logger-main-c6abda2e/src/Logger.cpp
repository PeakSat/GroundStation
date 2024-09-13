#include "FreeRTOS.h"
#include "task.h"
#include "UARTGatekeeperTask.hpp"
#include "Logger.hpp"

#define MaxLogNameSize 9
#define MaxTickCountStringSize 10

// allocate memory for the static variable
etl::format_spec Logger::format;


void Logger::log(Logger::LogLevel level, etl::istring &message) {
    etl::string<MaxLogNameSize> levelString;
    etl::string<MaxTickCountStringSize> time;

    if (level <= Logger::trace) {
        levelString.append("trace");
    } else if (level <= Logger::debug) {
        levelString.append("debug");
    } else if (level <= Logger::info) {
        levelString.append("info");
    } else if (level <= Logger::notice) {
        levelString.append("notice");
    } else if (level <= Logger::warning) {
        levelString.append("warning");
    } else if (level <= Logger::error) {
        levelString.append("error");
    } else {
        levelString = "emergency";
    }

    while (levelString.available()) {
        levelString.append(" ");
    }

    etl::to_string(xTaskGetTickCount(), time, format.width(MaxTickCountStringSize), 0);

    etl::string<LOGGER_MAX_MESSAGE_SIZE> output;
    output.append(time.c_str());
    output.append(" [");
    output.append(levelString.c_str());
    output.append("] ");

    etl::string<MaxLogNameSize> subsystemString = "COMMS";
    while (subsystemString.available()) {
        subsystemString.append(" ");
    }
    output.append(subsystemString.c_str());

    output.append(message.c_str());
    output.append("\n");


    if (uartGatekeeperTask) {
            uartGatekeeperTask->addToQueue(output);
    }


}
template<>
void convertValueToString(String<LOGGER_MAX_MESSAGE_SIZE>& message, float value) {
    etl::to_string(value, message, Logger::format.precision(Logger::MaxPrecision), true);
}

