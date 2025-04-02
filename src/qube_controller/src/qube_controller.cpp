#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class PidController
{
public:
	double update(double dt)
	{
		double error = setpoint - current;

		integral += error * dt;
		double derivative = (error - prevError) / dt;

		prevError = error;

		return kp * error + ki * integral + derivative * kd;
	}

protected:
	double kp = 0, ki = 0, kd = 0;
	double prevError = 0;
	double integral = 0;
	double setpoint = 0, current = 0;
};

class QubeControllerNode : public PidController, public rclcpp::Node
{
public:
	QubeControllerNode()
		: Node("qube_controller_node")
	{
		this->declare_parameter("kp", 1.0);
		this->declare_parameter("ki", 0.0);
		this->declare_parameter("kd", 0.0);
		this->declare_parameter("setpoint", 0);
		auto parameter_callback =
			[this](const std::vector<rclcpp::Parameter> &params)
			{
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
		post_set_parameters_handle = this->add_post_set_parameters_callback(parameter_callback);

		auto joint_state_listener = 
			[this](sensor_msgs::msg::JointState::UniquePtr msg)
			{
				current = msg->position.front();
			};
		state_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, joint_state_listener);
		cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/command", 10);
		wall_timer = create_wall_timer(std::chrono::milliseconds(20), [this]()
		{
			double pådrag = this->update(0.02);
			auto message = std_msgs::msg::Float64MultiArray();
			message.data.push_back(std::move(pådrag));
			//RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
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

