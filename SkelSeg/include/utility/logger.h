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
	INFO,	 /**< Informational messages. */
	WARNING, /**< Warning messages indicating potential issues. */
	ERROR	 /**< Error messages indicating failures. */
};

/**
 * @enum IndentLevel
 * @brief Specifies the severity level of the indent of a log message.
 */
enum class IndentLevel {
	ONE,  /**< No indent. */
	TWO,  /**< 15 spaces indent */
	THREE /**< One more arrow. */
};

/**
 * @enum LogLine
 * @brief Specifies the style of a separation line in the log.
 */
enum class LogLine {
	STAR,  /**< A line of asterisks. */
	SHARP, /**< A line of sharp signs. */
	DASH   /**< A line of dashes. */
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
	static Logger& Instance() {
		static Logger instance;
		return instance;
	}

	Logger(const Logger&) = delete;

	Logger& operator=(const Logger&) = delete;

	/**
	 * @brief Sets the log file path and opens the file for appending logs.
	 * @param log_file_path The path to the log file.
	 * @throws std::runtime_error If the log file cannot be opened.
	 */
	void SetLogFile(const std::filesystem::path& log_file_path) {
		std::lock_guard lock(file_mutex_);
		if (log_file_.is_open()) {
			log_file_.close();
		}
		log_file_.open(log_file_path, std::ios::app);
		if (!log_file_) {
			throw std::runtime_error(std::format("Cannot open log file ({})", log_file_path.string()));
		}
	}

	/**
	 * @brief Logs a message to both the console and the log file.
	 * @param message The message to log.
	 * @param log_level The log log_level, can be one of:
	 *   - `LogLevel::INFO`
	 *   - `LogLevel::WARNING`
	 *   - `LogLevel::ERROR`
	 * @param IndentLevel The indent level, can be one of:
	 *   - `IndentLevel::ONE`
	 *   - `IndentLevel::TWO`
	 *   - `IndentLevel::THREE`
	 * @param to_console Whether to output to the console.
	 * @param with_timestamp Whether to include a timestamp in the message.
	 * @throws std::runtime_error If the log log_level is `LogLevel::ERROR`.
	 */
	void Log(std::string_view message, LogLevel log_level = LogLevel::INFO, IndentLevel indent_level = IndentLevel::ONE, bool to_console = true,
			 bool with_timestamp = true) {
		std::string formatted_message = FormatMessage(message, log_level, indent_level, with_timestamp);

		// Output to log file if it is open
		{
			std::lock_guard lock(file_mutex_);
			if (log_file_.is_open()) {
				log_file_ << formatted_message;
			}
		}

		// Throw runtime_error if the log log_level is ERROR
		if (log_level == LogLevel::ERROR) {
			throw std::runtime_error(std::string(message));
		}

		// Output to console if required
		if (to_console) {
			OutputToConsole(formatted_message, log_level);
		}
	}

	/**
	 * @brief Adds a separation line to the log file and console output.
	 * @param line_style The line style, can be one of:
	 *   - `LogLine::STAR`
	 *   - `LogLine::DASH`
	 */
	void AddLine(LogLine line_style) {
		static constexpr std::string_view star_line = "****************************************************************\n";
		static constexpr std::string_view sharp_line = "################################################################\n";
		static constexpr std::string_view dash_line = "----------------------------------------------------------------\n";

		std::string_view message;

		switch (line_style) {
			case LogLine::STAR:
				message = star_line;
				break;
			case LogLine::SHARP:
				message = sharp_line;
				break;
			case LogLine::DASH:
				message = dash_line;
				break;
			default:
				message = "UNKNOWN";
				break;
		}

		{
			std::lock_guard lock(file_mutex_);
			if (log_file_.is_open()) {
				log_file_ << message;
			}
		}
		OutputToConsole(message, LogLevel::INFO);
	}

	/**
	 * @brief Adds a separation line to the log file and console output.
	 */
	void PrintTitle() {
		constexpr const char* title = R"(
   _____ _        _  _____
  / ____| |      | |/ ____|
 | (___ | | _____| | (___   ___  __ _
  \___ \| |/ / _ \ |\___ \ / _ \/ _` |
  ____) |   <  __/ |____) |  __/ (_| |
 |_____/|_|\_\___|_|_____/ \___|\__, |
                                __/ |
                               |___/

)";
		{
			std::lock_guard lock(file_mutex_);
			if (log_file_.is_open()) {
				log_file_ << title;
			}
		}

		OutputToConsole(title, LogLevel::INFO);
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
	 * @param IndentLevel The indent level, can be one of:
	 *   - `IndentLevel::ONE`
	 *   - `IndentLevel::TWO`
	 *   - `IndentLevel::THREE`
	 * @param with_timestamp Whether to include a timestamp.
	 * @return The formatted log message.
	 */
	static std::string FormatMessage(std::string_view message, LogLevel log_level, IndentLevel indent_level, bool with_timestamp) {
		if (indent_level == IndentLevel::ONE) {
			if (with_timestamp) {
				return std::format("[{}] [{}] {}\n", GetTimestamp(), LogLevelToString(log_level), message);
			}
			return std::format("[{}] {}\n", LogLevelToString(log_level), message);
		} else if (indent_level == IndentLevel::TWO) {
			if (with_timestamp) {
				return std::format("{}[{}] [{}] {}\n", IndentLevelToString(indent_level), GetTimestamp(), LogLevelToString(log_level), message);
			}
			return std::format("{}[{}] {}\n", IndentLevelToString(indent_level), LogLevelToString(log_level), message);
		} else {
			if (with_timestamp) {
				return std::format("               [{}] [{}] {}\n", GetTimestamp(), LogLevelToString(log_level), IndentLevelToString(indent_level),
								   message);
			}
			return std::format("               [{}] {}\n", LogLevelToString(log_level), IndentLevelToString(indent_level), message);
		}
	}

	/**
	 * @brief Gets the current timestamp in the format YYYY-MM-DD HH:MM:SS.mmm.
	 * @return The formatted timestamp string.
	 */
	static std::string GetTimestamp() {
		std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
		// Format time point with milliseconds precision
		return std::format("{0:%T}", floor<std::chrono::milliseconds>(now));
	}

	/**
	 * @brief Converts LogLevel to its string representation.
	 * @param log_level The log level, can be one of:
	 *   - `LogLevel::INFO`
	 *   - `LogLevel::WARNING`
	 *   - `LogLevel::ERROR`
	 * @return The string representation of the log level.
	 */
	static constexpr std::string_view LogLevelToString(LogLevel log_level) noexcept {
		switch (log_level) {
			case LogLevel::INFO:
				return "INFO";
			case LogLevel::WARNING:
				return "WARNING";
			case LogLevel::ERROR:
				return "ERROR";
			default:
				return "UNKNOWN";
		}
	}

	/**
	 * @brief Converts IndentLevel to its string representation.
	 * @param IndentLevel The indent level, can be one of:
	 *   - `IndentLevel::ONE`
	 *   - `IndentLevel::TWO`
	 *   - `IndentLevel::THREE`
	 * @return The string representation of the log level.
	 */
	static constexpr std::string_view IndentLevelToString(IndentLevel indent_level) noexcept {
		switch (indent_level) {
			case IndentLevel::ONE:
				return "";
			case IndentLevel::TWO:
				return "               ";
			case IndentLevel::THREE:
				return "--> ";
			default:
				return "UNKNOWN";
		}
	}

	/**
	 * @brief Outputs the formatted message to the console.
	 * @param message The message to output.
	 * @param log_level The log log_level, can be one of:
	 *   - `LogLevel::INFO`
	 *   - `LogLevel::WARNING`
	 *   - `LogLevel::ERROR`
	 */
	void OutputToConsole(std::string_view message, LogLevel log_level) {
		std::ostream& out_stream = log_level == LogLevel::WARNING ? std::cerr : std::cout;
		{
			std::lock_guard lock(console_mutex_);
			out_stream << message;
			out_stream.flush();
		}
	}

	mutable std::mutex file_mutex_;		// Mutex for thread-safe file operations
	mutable std::mutex console_mutex_;	// Mutex for thread-safe console output
	std::ofstream log_file_;			// Output file stream for logging
};


#endif	// LOGGER_H
