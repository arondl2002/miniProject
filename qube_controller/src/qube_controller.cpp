#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <algorithm>
#include <chrono>

class Timer
{
public:
	Timer()
	{
		now = std::chrono::steady_clock::now();
		old = now;
	}

	// Returns time since last called
	double getDt()
	{
		now = std::chrono::steady_clock::now();
		std::chrono::duration<double> diff = now - old;
		old = now;
		return diff.count();
	}

private:
	std::chrono::time_point<std::chrono::steady_clock> now, old;
};

class PidController
{
public:
	// Returns pid output
	double update()
	{
		double dt = timer.getDt();
		double error = setpoint - current;

		integral += error * dt;
		// Clamping integral effect
		integral = std::min(MINMAX_I, std::max(-MINMAX_I, integral));

		double derivative = (error - prevError) / dt;

		prevError = error;

		double output = kp * error + ki * integral + derivative * kd;

		// Clamping output value
		output = std::min(MINMAX_OUTPUT, std::max(-MINMAX_OUTPUT, output));

		return output;
	}

protected:
	// Absolute value limits for integral windup
	static constexpr double MINMAX_I = 10;
	// Absolute value limits for total output value
	static constexpr double MINMAX_OUTPUT = 100;

	double kp = 0, ki = 0, kd = 0;
	double prevError = 0;
	double integral = 0;
	double setpoint = 0, current = 0;
	Timer timer;
};

// Wraps PidController and rclcpp::Node together in a class
class QubeControllerNode : public PidController, public rclcpp::Node
{
public:
	QubeControllerNode()
		: Node("qube_controller_node")
	{
		// Parameter callback function
		auto parameter_callback =
			[this](const std::vector<rclcpp::Parameter> &params)
			{
				// Loop through and set the correct param
				for (const auto &param : params)
				{
					if (param.get_name() == "kp")
					{
						this->kp = param.as_double();
						RCLCPP_INFO(this->get_logger(), "kp was set to: %f", param.as_double());
					}
					else if (param.get_name() == "ki")
					{
						this->ki = param.as_double();
						RCLCPP_INFO(this->get_logger(), "ki was set to: %f", param.as_double());
					}
					else if (param.get_name() == "kd")
					{
						this->kd = param.as_double();
						RCLCPP_INFO(this->get_logger(), "kd was set to: %f", param.as_double());
					}
					else if (param.get_name() == "setpoint")
					{
						this->setpoint = param.as_double();
						RCLCPP_INFO(this->get_logger(), "Setpoint was set to: %f", param.as_double());
					}
				}
			};
		// Attach parameter set callback
		post_set_parameters_handle = this->add_post_set_parameters_callback(parameter_callback);
		this->declare_parameter("kp", 23.0);
		this->declare_parameter("ki", 0.294);
		this->declare_parameter("kd", 0.05);
		this->declare_parameter("setpoint", 0.0);

		// Joint State Subscriber callback function
		auto joint_state_listener = 
			[this](sensor_msgs::msg::JointState::UniquePtr msg)
			{
				current = msg->position.front();
			};
		// Attaching callback to /joint_states
		state_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, joint_state_listener);
		// Attach publisher to /velocity_controller/commands
		cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
		wall_timer = create_wall_timer(std::chrono::milliseconds(20), [this]()
		{
			double output = this->update();
			auto message = std_msgs::msg::Float64MultiArray();
			message.data.push_back(std::move(output));
			// Publishing output value
			this->cmd_pub->publish(message);
		});
	}
private:
	rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_parameters_handle;
	rclcpp::TimerBase::SharedPtr wall_timer;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<QubeControllerNode>());
	rclcpp::shutdown();
	return 0;
}

