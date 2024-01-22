#if ROS1_BUILD
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "f1tenth_race/pwm_high.h"
#include "f1tenth_race/drive_values.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#elif ROS2_BUILD
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "teensy_drive_msgs/msg/pwm_high.hpp"
#include "teensy_drive_msgs/msg/drive_values.hpp"
//#include "std_msgs/msg/int32multiarray.hpp"
//#include "std_msgs/msg/multiarraydimension.hpp"
#endif // ROS2_BUILD

#include <string>
#include <thread>
#include "protocol.h"
#include "serial_port.hpp"
#include <math.h>

#include <unistd.h>

#if ROS1_BUILD
#define debug(x) ROS_DEBUG(x)
#define INFO(...) ROS_INFO(__VA_ARGS__)
#define WARN(...) ROS_WARN(__VA_ARGS__)
#define ERROR(...) ROS_ERROR(__VA_ARGS__)
#define FATAL(...) ROS_ERROR(__VA_ARGS__)
#elif ROS2_BUILD
#include "utils.h"
#define INFO(...) RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define WARN(...) RCLCPP_WARN(get_logger(), __VA_ARGS__)
#define ERROR(...) RCLCPP_FATAL(get_logger(), __VA_ARGS__)
#define FATAL(...) RCLCPP_FATAL(get_logger(), __VA_ARGS__)
#endif // else ROS2_BUILD

// poll, POLLERR, POLLHUP, POLLIN
#include <poll.h>

#if ROS1_BUILD
class TeensyDrive {
#elif ROS2_BUILD
using namespace std::chrono_literals;

class TeensyDrive : public rclcpp::Node {
#endif // ROS2_BUILD

public:

#if ROS1_BUILD
	TeensyDrive(ros::NodeHandle *n) {
		std::string port;
		n->getParam("port", port);
#elif ROS2_BUILD
	TeensyDrive()
		: Node("teensy_drive") {

		rcl_interfaces::msg::ParameterDescriptor port_param_desc;
		port_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
		port_param_desc.read_only = true;
		port_param_desc.description = "teensy-drive device port name";

		std::string port = declare_parameter<std::string>("port", "", port_param_desc);
#endif // ROS2_BUILD

		// attempt to connect to the serial port
		try {
			serial_port_.open(port);
		} catch (const std::runtime_error &e) {
			ERROR("failed to open the serial port: %s", e.what());
#if ROS2_BUILD
			rclcpp::shutdown();
#endif // ROS2_BUILD
			return;
		}

#if ROS1_BUILD
		pwm_high_publisher_ = n->advertise<f1tenth_race::pwm_high>(
			"/pwm_high",
			1
		);

		estop_publisher_ = n->advertise<std_msgs::Bool>(
			"/eStop",
			1
		);

		encoder_publisher_ = n->advertise<std_msgs::Int32MultiArray>(
			"/sensors/wheel",
			1
		);

		msg_encoder_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg_encoder_.layout.dim[0].size = 4;
		msg_encoder_.layout.dim[0].stride = 1;
		msg_encoder_.layout.dim[0].label = "position";

		msg_encoder_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg_encoder_.layout.dim[1].size = 4;
		msg_encoder_.layout.dim[1].stride = 1;
		msg_encoder_.layout.dim[1].label = "speed";

		msg_encoder_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg_encoder_.layout.dim[2].size = 4;
		msg_encoder_.layout.dim[2].stride = 1;
		msg_encoder_.layout.dim[2].label = "speed2";

		msg_encoder_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg_encoder_.layout.dim[3].size = 1;
		msg_encoder_.layout.dim[3].stride = 1;
		msg_encoder_.layout.dim[3].label = "speed_vehicle";

		sub_estop = n->subscribe<std_msgs::Bool>(
			"/eStop",
			1,
			&TeensyDrive::estop_callback, this
		);

		sub_drive_pwm = n->subscribe<f1tenth_race::drive_values>(
			"/drive_pwm",
			1,
			&TeensyDrive::drive_pwm_callback, this
		);

#elif ROS2_BUILD
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
		/*encoder_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
			"/sensors/wheel",
			1
		);*/
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
#endif // ROS2_BUILD
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

#if ROS1_BUILD
	~TeensyDrive() {
#elif ROS2_BUILD
	~TeensyDrive() override {
#endif // ROS2_BUILD

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
		set_packet_handler(
			MESSAGE_VERSION,
			static_cast<packet_handler>([](const union packet *packet, void *context) {
				auto _this = reinterpret_cast<TeensyDrive *>(context);
				_this->handle_version_packet(reinterpret_cast<const struct packet_message_version *>(packet));
			}),
			(void *) this
		);
		set_packet_handler(
			MESSAGE_ENCODER,
			static_cast<packet_handler>([](const union packet *packet, void *context) {
				auto _this = reinterpret_cast<TeensyDrive *>(context);
				_this->handle_encoder_packet(reinterpret_cast<const struct packet_message_encoder *>(packet));
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

		struct packet_message_version packet{
			MESSAGE_VERSION,
			sizeof(struct packet_message_version),
			{"VERSION"},
			0,
		};
		send_packet(serial_port_.getFd(), reinterpret_cast<union packet *>(&packet));

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
#if ROS1_BUILD
		pwm_high_publisher_.publish(msg_pwm_high_);
#elif ROS2_BUILD
		pwm_high_publisher_->publish(msg_pwm_high_);
#endif // ROS2_BUILD
	}

	void handle_estop_packet(const struct packet_message_bool *msg) {
		INFO("handle_estop_msg: data=%d", msg->payload.data);
		msg_estop_.data = msg->payload.data;
#if ROS1_BUILD
		estop_publisher_.publish(msg_estop_);
#elif ROS2_BUILD
		estop_publisher_->publish(msg_estop_);
#endif // ROS2_BUILD
	}

	void handle_version_packet(const struct packet_message_version *msg) {
		WARN("Starting Teensy -- FW build %s", msg->payload.data);
	}

#if ROS1_BUILD
	void estop_callback(const std_msgs::Bool::ConstPtr &msg) const {
#elif ROS2_BUILD
	void estop_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) const {
#endif // ROS2_BUILD
		INFO("estop_callback: data=%d", msg->data);
		struct packet_message_bool packet{
			MESSAGE_ESTOP,
			sizeof(struct packet_message_bool),
			{msg->data},
			0,
		};
		send_packet(serial_port_.getFd(), reinterpret_cast<union packet *>(&packet));
	}

	void handle_encoder_packet(const struct packet_message_encoder *msg) {
		//INFO("handle_encoder_msg: fl=%d fr=%d rl=%d rr=%d fl=%d fr=%d rl=%d rr=%d", msg->payload.fl_speed, msg->payload.fr_speed, msg->payload.rl_speed, msg->payload.rr_speed, msg->payload.fl_speed2, msg->payload.fr_speed2, msg->payload.rl_speed2, msg->payload.rr_speed2);
		int car_speed = 0;
		//                    us->s   2 * teeth     1wheel rotation[m]  m->mm
		int conversion_ratio = 1e6  /  (2 * 30)  * (2 * M_PI * 0.055) *  1e3 ;
		msg_encoder_.data.clear();
		msg_encoder_.data.emplace_back(msg->payload.fl_position);
		msg_encoder_.data.emplace_back(msg->payload.fr_position);
		msg_encoder_.data.emplace_back(msg->payload.rl_position);
		msg_encoder_.data.emplace_back(msg->payload.rr_position);
		if (msg->payload.fl_speed != 0) {
			msg_encoder_.data.emplace_back(msg->payload.fl_speed);
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		if (msg->payload.fr_speed != 0) {
			msg_encoder_.data.emplace_back(msg->payload.fr_speed);
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		if (msg->payload.rl_speed != 0) {
			msg_encoder_.data.emplace_back(msg->payload.rl_speed);
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		if (msg->payload.rr_speed != 0) {
			msg_encoder_.data.emplace_back(msg->payload.rr_speed);
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		if (msg->payload.fl_speed2 != 0) {
			msg_encoder_.data.emplace_back(msg->payload.fl_speed2);
			car_speed += msg->payload.fl_speed2;
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		if (msg->payload.fr_speed2 != 0) {
			msg_encoder_.data.emplace_back(msg->payload.fr_speed2);
			car_speed += msg->payload.fr_speed2;
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		if (msg->payload.rl_speed2 != 0) {
			msg_encoder_.data.emplace_back(msg->payload.rl_speed2);
			car_speed += msg->payload.rl_speed2;
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		if (msg->payload.rr_speed2 != 0) {
			msg_encoder_.data.emplace_back(msg->payload.rr_speed2);
			car_speed += msg->payload.rr_speed2;
		} else {
			msg_encoder_.data.emplace_back(0);
		}
		msg_encoder_.data.emplace_back(car_speed / 4);
#if ROS1_BUILD
		encoder_publisher_.publish(msg_encoder_);
#elif ROS2_BUILD
		//encoder_publisher_->publish(msg_encoder_);
#endif // ROS2_BUILD
	}

#if ROS1_BUILD
	void drive_pwm_callback(const f1tenth_race::drive_values::ConstPtr &msg) const {
#elif ROS2_BUILD
	void drive_pwm_callback(const teensy_drive_msgs::msg::DriveValues::ConstSharedPtr &msg) const {
#endif // ROS2_BUILD
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

#if ROS2_BUILD
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
#endif // ROS2_BUILD


#if ROS1_BUILD
	// publishers
	ros::Publisher pwm_high_publisher_;
	ros::Publisher estop_publisher_;
	ros::Publisher encoder_publisher_;

	// pre-allocated messages to publish
	std_msgs::Bool msg_estop_;
	f1tenth_race::pwm_high msg_pwm_high_;
	std_msgs::Int32MultiArray msg_encoder_;

	// subscriptions
	    ros::Subscriber sub_estop;
	    ros::Subscriber sub_drive_pwm;
#elif ROS2_BUILD
	// rclcpp::TimerBase::SharedPtr timer_;

	// publishers
	rclcpp::Publisher<teensy_drive_msgs::msg::PwmHigh>::SharedPtr pwm_high_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;
	//rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_publisher_;

	// pre-allocated messages to publish
	teensy_drive_msgs::msg::PwmHigh msg_pwm_high_;
	std_msgs::msg::Bool msg_estop_;
	//std_msgs::msg::Int32MultiArray msg_encoder_;

	// subscriptions
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
	rclcpp::Subscription<teensy_drive_msgs::msg::DriveValues>::SharedPtr drive_pwm_subscription_;

	// parameters change callback
	OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
#endif // ROS2_BUILD

	SerialPort serial_port_;
	// serial port packet receiving thread
	volatile bool packet_thread_run_;
	std::unique_ptr<std::thread> packet_thread_;

};

int main(int argc, char *argv[]) {

#if ROS1_BUILD
	ros::init(argc, argv, "teensy_drive");

	ros::NodeHandle n("~");
	std::shared_ptr<TeensyDrive> t = std::make_shared<TeensyDrive>(&n);
	ros::spin();
#elif ROS2_BUILD
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TeensyDrive>());
	rclcpp::shutdown();
#endif // ROS2_BUILD

	return 0;

}
