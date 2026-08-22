#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>



namespace controller::mppi_controller
{
/**
 * \brief `mppi_controller`'s own thin `rclcpp::Logger` wrapper (colored
 * error/success output, parent/child logger construction).
 *
 * \note Distinct from \c mpl::rclcpp_utils::Logger used elsewhere in this
 * codebase (`agent_sim`, `motion_model_base`, `pid_controller`, ...) --
 * this package keeps its own copy rather than depending on `project_utils`,
 * consistent with it being a self-contained port of nav2's mppi_controller.
 */
class Logger
{
  public:
	// ANSI color codes
	static constexpr const char * RED_COLOR   = "\033[1;31m";
	static constexpr const char * GREEN_COLOR = "\033[1;32m";
	static constexpr const char * RESET_COLOR = "\033[0m";

	Logger(const std::string & logggerName, bool useColoredError = true)
	    : ros2_logger_(rclcpp::get_logger(logggerName)),
	      use_colored_error_(useColoredError)
	{
	}

	Logger(bool useColoredError = true)
	    : ros2_logger_(rclcpp::get_logger("controller")),
	      use_colored_error_(useColoredError)
	{
	}

	// Root logger
	explicit Logger(rclcpp::Logger logger, bool useColoredError = true)
	    : ros2_logger_(logger),
	      use_colored_error_(useColoredError)
	{
	}

	// Child logger from rclcpp logger
	Logger(
	    rclcpp::Logger logger,
	    const std::string & child,
	    bool useColoredError = true)
	    : ros2_logger_(logger.get_child(child)),
	      use_colored_error_(useColoredError)
	{
	}

	// Child logger from another Logger
	Logger(
	    Logger & parent,
	    const std::string & child,
	    bool useColoredError = true)
	    : ros2_logger_(parent.ros2_logger_.get_child(child)),
	      use_colored_error_(useColoredError)
	{
	}

	template <typename... Args>
	void info(const char * format, Args... args) const
	{
		RCLCPP_INFO(ros2_logger_, format, args...);
	}

	template <typename... Args>
	void success(const char * format, Args... args) const
	{
		std::string colored_format =
		    std::string(GREEN_COLOR) + format + RESET_COLOR;
		RCLCPP_INFO(ros2_logger_, colored_format.c_str(), args...);
	}

	template <typename... Args>
	void warn(const char * format, Args... args) const
	{
		RCLCPP_WARN(ros2_logger_, format, args...);
	}

	template <typename... Args>
	void error(const char * format, Args... args) const
	{
		if (use_colored_error_) {
			std::string colored_format =
			    std::string(RED_COLOR) + format + RESET_COLOR;

			RCLCPP_ERROR(
			    ros2_logger_,
			    colored_format.c_str(),
			    args...);
		}
		else {
			RCLCPP_ERROR(ros2_logger_, format, args...);
		}
	}

	template <typename... Args>
	void info_throttle(
	    rclcpp::Clock & clock,
	    int64_t duration_ms,
	    const char * format,
	    Args... args) const
	{
		RCLCPP_INFO_THROTTLE(
		    ros2_logger_,
		    clock,
		    duration_ms,
		    format,
		    args...);
	}

	const rclcpp::Logger & getRos2Logger() const { return ros2_logger_; }

  private:
	rclcpp::Logger ros2_logger_;
	bool use_colored_error_;
};


} // namespace controller::mppi_controller