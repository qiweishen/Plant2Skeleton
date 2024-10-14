#ifndef TIMER_H
#define TIMER_H


#include <chrono>



/**
 * @class Timer
 * @brief A high-resolution timer for measuring elapsed time.
 *
 * The Timer class provides functionality to measure elapsed time with high precision.
 * It supports various time units and is suitable for performance measurements.
 */
class Timer {
public:
	using Clock = std::chrono::steady_clock;

	/**
	 * @brief Constructs a new Timer and starts it.
	 */
	Timer() noexcept : start_point_{ Clock::now() } {}

	/**
	 * @brief Resets the timer to the current time.
	 */
	void reset() noexcept { start_point_ = Clock::now(); }

	/**
	 * @enum TimeUnit
	 * @brief Specifies the unit of time for elapsed time measurement.
	 */
	enum class TimeUnit {
		Nanoseconds,  /**< Nanoseconds */
		Microseconds, /**< Microseconds */
		Milliseconds, /**< Milliseconds */
		Seconds,	  /**< Seconds */
		Minutes		  /**< Minutes */
	};

	/**
	 * @brief Gets the elapsed time since the timer was started or last reset.
	 * @tparam Unit The time unit to return the elapsed time in.
	 * @return The elapsed time in the specified time unit.
	 *
	 * @note The function is marked as [[nodiscard]] to encourage checking the return value.
	 */
	template<TimeUnit Unit = TimeUnit::Milliseconds>
	[[nodiscard]] double elapsed() const noexcept {
		const auto duration = Clock::now() - start_point_;
		return duration_cast<Unit, double>(duration);
	}


private:
	Clock::time_point start_point_;

	template<TimeUnit Unit, typename T>
	static T duration_cast(const Clock::duration& duration) noexcept {
		if constexpr (Unit == TimeUnit::Nanoseconds) {
			return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
		} else if constexpr (Unit == TimeUnit::Microseconds) {
			return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
		} else if constexpr (Unit == TimeUnit::Seconds) {
			return std::chrono::duration_cast<std::chrono::duration<T>>(duration).count();
		} else if constexpr (Unit == TimeUnit::Minutes) {
			return std::chrono::duration_cast<std::chrono::duration<T, std::ratio<60>>>(duration).count();
		} else {
			return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
		}
	}
};



#endif	// TIMER_H
