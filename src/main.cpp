#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "f1tenth_race/msg/pwm_high.hpp"
#include "f1tenth_race/msg/drive_values.hpp"

#include "protocol.h"

using namespace std::chrono_literals;

class TeensyDrive : public rclcpp::Node {

public:

	TeensyDrive()
		: Node("teensy_drive") {

		rcl_interfaces::msg::ParameterDescriptor port_param_desc;
		port_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
		port_param_desc.description = "teensy-drive device port name";

		std::string port = declare_parameter<std::string>("port", "", port_param_desc);

		// see https://roboticsbackend.com/ros2-rclcpp-parameter-callback/
		parameters_callback_handle_ = add_on_set_parameters_callback(
			[this](const auto &p) { return parameters_callback(p); }
		);

		pwm_high_publisher_ = create_publisher<f1tenth_race::msg::PwmHigh>(
			"/pwm_high",
			1
		);
		estop_subscription_ = create_subscription<std_msgs::msg::Bool>(
			"/eStop",
			1,
			// NOLINTNEXTLINE(performance-unnecessary-value-param)
			[this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
				estop_callback(msg);
			}
		);
		drive_pwm_subscription_ = create_subscription<f1tenth_race::msg::DriveValues>(
			"/drive_pwm",
			1,
			// NOLINTNEXTLINE(performance-unnecessary-value-param)
			[this](const f1tenth_race::msg::DriveValues::ConstSharedPtr msg) {
				drive_pwm_callback(msg);
			}
		);
		timer_ = this->create_wall_timer(
			500ms,
			[this] { timer_callback(); }
		);

	}

private:

	void estop_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) const {
		RCLCPP_INFO(get_logger(), "estop_callback: data=%d", msg->data);
	}

	void drive_pwm_callback(const f1tenth_race::msg::DriveValues::ConstSharedPtr &msg) const {
		RCLCPP_INFO(
			get_logger(),
			"drive_pwm_callback: pwm_drive=%d pwm_angle=%d",
			msg->pwm_drive, msg->pwm_angle
		);
	}

	void timer_callback() {
		RCLCPP_INFO(get_logger(), "timer_callback");
	}

	// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
	rcl_interfaces::msg::SetParametersResult parameters_callback(
		const std::vector<rclcpp::Parameter> &parameters
	) {

		// see comments for reconfigure_callback
		// in ws/src/decision_and_control/follow_the_gap_v0_ride/follow_the_gap_v0_ride/ride_node.py

		rcl_interfaces::msg::SetParametersResult result;
		result.successful = false;
		result.reason = "Changing parameters during runtime is not supported yet.";

		return result;

	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<f1tenth_race::msg::PwmHigh>::SharedPtr pwm_high_publisher_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
	rclcpp::Subscription<f1tenth_race::msg::DriveValues>::SharedPtr drive_pwm_subscription_;
	OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

};

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TeensyDrive>());
	rclcpp::shutdown();

	return 0;

}
