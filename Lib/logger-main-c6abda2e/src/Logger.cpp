#include <Logger.hpp>

etl::format_spec Logger::format;

Logger::LogEntry::LogEntry(LogLevel level) : level(level) {}

Logger::LogEntry::~LogEntry() {
	// When the destructor is called, the log message is fully "designed". Now we can finally "display" it to the user.
	Logger::log(level, message);
}

template<>
void convertValueToString(String<LOGGER_MAX_MESSAGE_SIZE>& message, float value) {
    etl::to_string(value, message, Logger::format.precision(Logger::MaxPrecision), true);
}
