#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "teensy_drive_msgs/msg/pwm_high.hpp"
#include "teensy_drive_msgs/msg/drive_values.hpp"

#include "protocol.h"
#include "serial_port.hpp"

#include <unistd.h>

#include "utils.h"

#include <poll.h>

using namespace std::chrono_literals;

class TeensyDrive : public rclcpp::Node {

public:

	TeensyDrive()
		: Node("teensy_drive") {

		rcl_interfaces::msg::ParameterDescriptor port_param_desc;
		port_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
		port_param_desc.read_only = true;
		port_param_desc.description = "teensy-drive device port name";

		std::string port = declare_parameter<std::string>("port", "", port_param_desc);

		// attempt to connect to the serial port
		try {
			serial_port_.open(port);
		} catch (const std::runtime_error &e) {
			RCLCPP_FATAL(get_logger(), "failed to open the serial port: %s", e.what());
			rclcpp::shutdown();
			return;
		}

		// see https://roboticsbackend.com/ros2-rclcpp-parameter-callback/
		parameters_callback_handle_ = add_on_set_parameters_callback(
			[this](const auto &p) { return parameters_callback(p); }
		);

		pwm_high_publisher_ = create_publisher<teensy_drive_msgs::msg::PwmHigh>(
			"/pwm_high",
			1
		);
		estop_publisher_ = create_publisher<std_msgs::msg::Bool>(
			"/eStop",
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
		drive_pwm_subscription_ = create_subscription<teensy_drive_msgs::msg::DriveValues>(
			"/drive_pwm",
			1,
			// NOLINTNEXTLINE(performance-unnecessary-value-param)
			[this](const teensy_drive_msgs::msg::DriveValues::ConstSharedPtr msg) {
				drive_pwm_callback(msg);
			}
		);
		// timer_ = this->create_wall_timer(
		// 	500ms,
		// 	[this] { timer_callback(); }
		// );

		packet_thread_run_ = true;
		packet_thread_ = std::make_unique<std::thread>(
			[this]() {
				receive_packets();
			}
		);

	}

	~TeensyDrive() override {

		debug("TeensyDrive destructor");

		if (packet_thread_) {
			packet_thread_run_ = false;
			packet_thread_->join();
		}

	}

private:

	void receive_packets() {
		debug("receive_packets");

		set_packet_handler(
			MESSAGE_PWM_HIGH,
			static_cast<packet_handler>([](const union packet *packet, void *context) {
				auto _this = reinterpret_cast<TeensyDrive *>(context);
				_this->handle_pwm_high_packet(reinterpret_cast<const struct packet_message_pwm_high *>(packet));
			}),
			(void *) this
		);
		set_packet_handler(
			MESSAGE_ESTOP,
			static_cast<packet_handler>([](const union packet *packet, void *context) {
				auto _this = reinterpret_cast<TeensyDrive *>(context);
				_this->handle_estop_packet(reinterpret_cast<const struct packet_message_bool *>(packet));
			}),
			(void *) this
		);
		enum {
			DEV
		};
		// struct pollfd fds[1] = {
		// 	[DEV]   = {.fd = serial_port_.getFd(), .events = POLLIN},
		// };
		struct pollfd fds[1] = {
			{
				serial_port_.getFd(),
				POLLIN,
				0,
			},
		};

		int result;

		unsigned char buffer[4095];

		while (packet_thread_run_) {

			result = poll(fds, 1, 1000);

			if (result == 0) {
				// printf("poll timeout expired\n");
				continue;
			}

			if (result == -1) {
				if (errno == EINTR) {
					// placing debugger point sends signal
					continue;
				}
				// printf("poll failed: %s\n", strerror(errno));
				debug("poll failed");
				break;
			}

			if (fds[DEV].revents & POLLERR) {
				debug("poll DEV POLLERR");
			}

			if (fds[DEV].revents & POLLHUP) {
				debug("poll DEV POLLHUP\n");
				debug("device disconnected\n");
				break;
			}

			if (fds[DEV].revents & POLLIN) {

				result = (int) ::read(serial_port_.getFd(), buffer, sizeof(buffer));

				if (result > 0) {
					// debug_printf("read %d bytes from device\n", result);
					// print_bytes(buffer, result);
					process_messages(buffer, result);
				} else {
					// debug_printf("read result: %d\n", result);
					// result == 0 -> EOF (but this should not happen as in such a case POLLHUP event should be emitted)
					// result == -1 -> error
					debug("read failed");
					break;
				}

			}

		}
	}

	void handle_pwm_high_packet(const struct packet_message_pwm_high *msg) {
		// RCLCPP_INFO(
		// 	get_logger(),
		// 	"handle_pwm_high_msg: period_thr=%d period_str=%d\n",
		// 	(int) msg->payload.period_thr, (int) msg->payload.period_str
		// );
		msg_pwm_high_.period_thr = msg->payload.period_thr;
		msg_pwm_high_.period_str = msg->payload.period_str;
		pwm_high_publisher_->publish(msg_pwm_high_);
	}

	void handle_estop_packet(const struct packet_message_bool *msg) {
		RCLCPP_INFO(get_logger(), "handle_estop_msg: data=%d", msg->payload.data);
		msg_estop_.data = msg->payload.data;
		estop_publisher_->publish(msg_estop_);
	}

	void estop_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) const {
		RCLCPP_INFO(get_logger(), "estop_callback: data=%d", msg->data);
		struct packet_message_bool packet{
			MESSAGE_ESTOP,
			sizeof(struct packet_message_bool),
			{msg->data},
			0,
		};
		send_packet(serial_port_.getFd(), reinterpret_cast<union packet *>(&packet));
	}

	void drive_pwm_callback(const teensy_drive_msgs::msg::DriveValues::ConstSharedPtr &msg) const {
		// RCLCPP_DEBUG(
		// 	get_logger(),
		// 	"drive_pwm_callback: pwm_drive=%d pwm_angle=%d",
		// 	msg->pwm_drive, msg->pwm_angle
		// );
		struct packet_message_drive_values packet{
			MESSAGE_DRIVE_PWM,
			sizeof(struct packet_message_drive_values),
			{
				msg->pwm_drive,
				msg->pwm_angle,
			},
			0,
		};
		send_packet(serial_port_.getFd(), reinterpret_cast<union packet *>(&packet));
	}

	// void timer_callback() {
	// 	RCLCPP_INFO(get_logger(), "timer_callback");
	// 	// pwm_high_publisher_->publish(msg_pwm_high_);
	// }

	// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
	rcl_interfaces::msg::SetParametersResult parameters_callback(
		const std::vector<rclcpp::Parameter> &/* parameters */
	) {

		// see comments for reconfigure_callback
		// in ws/src/decision_and_control/follow_the_gap_v0_ride/follow_the_gap_v0_ride/ride_node.py

		rcl_interfaces::msg::SetParametersResult result;
		result.successful = false;
		result.reason = "Changing parameters during runtime is not supported yet.";

		return result;

	}

	// rclcpp::TimerBase::SharedPtr timer_;

	// publishers
	rclcpp::Publisher<teensy_drive_msgs::msg::PwmHigh>::SharedPtr pwm_high_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;

	// pre-allocated messages to publish
	teensy_drive_msgs::msg::PwmHigh msg_pwm_high_;
	std_msgs::msg::Bool msg_estop_;

	// subscriptions
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
	rclcpp::Subscription<teensy_drive_msgs::msg::DriveValues>::SharedPtr drive_pwm_subscription_;

	// parameters change callback
	OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

	SerialPort serial_port_;
	// serial port packet receiving thread
	volatile bool packet_thread_run_;
	std::unique_ptr<std::thread> packet_thread_;

};

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TeensyDrive>());
	rclcpp::shutdown();

	return 0;

}
