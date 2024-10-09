#ifndef LOGGER_H
#define LOGGER_H


#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>



/**
 * @enum LogLevel
 * @brief Specifies the severity level of a log message.
 */
enum class LogLevel {
    NONE, /**< No log level, used internally. */
    INFO, /**< Informational messages. */
    WARNING, /**< Warning messages indicating potential issues. */
    ERROR /**< Error messages indicating failures. */
};

/**
 * @enum LogLine
 * @brief Specifies the style of a separation line in the log.
 */
enum class LogLine {
    STAR, /**< A line of asterisks. */
    DASH /**< A line of dashes. */
};


/**
 * @class Logger
 * @brief A thread-safe singleton logger for logging messages to a file and console.
 *
 * The Logger class provides thread-safe logging functionality. It allows logging messages
 * with different severity levels, adding separation lines, and supports output to both
 * console and log file.
 */
class Logger {
public:
    /**
     * @brief Retrieves the singleton instance of Logger.
     * @return Reference to the Logger instance.
     */
    static Logger &Instance() {
        static Logger instance;
        return instance;
    }

    Logger(const Logger &) = delete;

    Logger &operator=(const Logger &) = delete;

    /**
     * @brief Sets the log file path and opens the file for appending logs.
     * @param log_file_path The path to the log file.
     * @throws std::runtime_error If the log file cannot be opened.
     */
    void SetLogFile(const std::filesystem::path &log_file_path) {
        std::lock_guard lock(file_mutex_);
        if (log_file_.is_open()) {
            log_file_.close();
        }
        log_file_.open(log_file_path, std::ios::app);
        if (!log_file_) {
            throw std::runtime_error("Cannot open log file: " + log_file_path.string());
        }
    }

    /**
     * @brief Logs a message to both the console and the log file.
     * @param message The message to log.
     * @param level The log level, can be one of:
     *   - `LogLevel::INFO`
     *   - `LogLevel::WARNING`
     *   - `LogLevel::ERROR`
     * @param to_console Whether to output to the console.
     * @param with_timestamp Whether to include a timestamp in the message.
     * @throws std::runtime_error If the log level is `LogLevel::ERROR`.
     */
    void Log(std::string_view message, LogLevel level = LogLevel::INFO, bool to_console = true,
             bool with_timestamp = true) {
        std::string formatted_message = FormatMessage(message, level, with_timestamp);

        // Output to log file if it is open
        {
            std::lock_guard lock(file_mutex_);
            if (log_file_.is_open()) {
                log_file_ << formatted_message;
            }
        }

        // Throw runtime_error if the log level is ERROR
        if (level == LogLevel::ERROR) {
            throw std::runtime_error(std::string(message));
        }

        // Output to console if required
        if (to_console) {
            OutputToConsole(formatted_message, level);
        }
    }

    /**
     * @brief Adds a separation line to the log file and console output.
     * @param line_style The line style, can be one of:
     *   - `LogLine::STAR`
     *   - `LogLine::DASH`
     */
    void AddLine(LogLine line_style) {
        constexpr std::string_view star_line = "****************************************************************\n";
        constexpr std::string_view dash_line = "----------------------------------------------------------------\n";

        std::string_view message = (line_style == LogLine::STAR) ? star_line : dash_line; {
            std::lock_guard lock(file_mutex_);
            if (log_file_.is_open()) {
                log_file_ << message;
            }
        }
        OutputToConsole(message, LogLevel::NONE);
    }

private:
    Logger() = default;

    ~Logger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    /**
    * @brief Formats the log message with optional timestamp and log level.
    * @param message The message to format.
    * @param level The log level, can be one of:
    *   - `LogLevel::INFO`
    *   - `LogLevel::WARNING`
    *   - `LogLevel::ERROR`
    * @param with_timestamp Whether to include a timestamp.
    * @return The formatted log message.
    */
    static std::string FormatMessage(std::string_view message, LogLevel level, bool with_timestamp) {
        if (with_timestamp) {
            return std::format("[{}] [{}] {}\n", GetTimestamp(), ToString(level), message);
        } else {
            return std::format("[{}] {}\n", ToString(level), message);
        }
    }

    /**
     * @brief Gets the current timestamp in the format YYYY-MM-DD HH:MM:SS.mmm.
     * @return The formatted timestamp string.
     */
    static std::string GetTimestamp() {
        using namespace std::chrono;
        auto now = system_clock::now();
        // Format time point with milliseconds precision
        return std::format("{:%Y-%m-%d %H:%M:%S.%f}", floor<milliseconds>(now));
    }

    /**
    * @brief Converts LogLevel to its string representation.
    * @param level The log level, can be one of:
    *   - `LogLevel::INFO`
    *   - `LogLevel::WARNING`
    *   - `LogLevel::ERROR`
    * @return The string representation of the log level.
    */
    static constexpr std::string_view ToString(LogLevel level) noexcept {
        switch (level) {
            case LogLevel::NONE: return "";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    /**
    * @brief Outputs the formatted message to the console.
    * @param message The message to output.
    * @param level The log level.
    * @param level The log level, can be one of:
    *   - `LogLevel::INFO`
    *   - `LogLevel::WARNING`
    *   - `LogLevel::ERROR`
    */
    void OutputToConsole(std::string_view message, LogLevel level) {
        std::ostream &out_stream = (level == LogLevel::ERROR) ? std::cerr : std::cout; {
            std::lock_guard lock(console_mutex_);
            out_stream << message;
            out_stream.flush();
        }
    }

    mutable std::mutex file_mutex_; /**< Mutex for thread-safe file operations. */
    mutable std::mutex console_mutex_; /**< Mutex for thread-safe console output. */
    std::ofstream log_file_; /**< Output file stream for logging. */
};



#endif // LOGGER_H
