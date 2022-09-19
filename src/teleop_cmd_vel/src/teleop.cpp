#include <cstdio>
#include <memory>
#include <chrono>
#include <assert.h>

#include <conio.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;


class Teleop : public rclcpp::Node{

	public:

		Teleop() : Node("teleop"){
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", default_qos);

			timer_key_read = this->create_wall_timer(100ms, std::bind(&Teleop::readKey, this));


			RCLCPP_INFO(this->get_logger(), "ECAi21 | Turtle Teleop");

		}

		~Teleop(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}

	private:

		void readKey(){

			auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();

			if (kbhit()){
				c = getch();

				RCLCPP_INFO(this->get_logger(), "Pressed key %c",c);
        	}

			switch (c)
			{
			case 'w':
				cmd_msg->linear.x = 1;
				cmd_msg->angular.z = 0;
				break;
			case 's':
				cmd_msg->linear.x = -1;
				cmd_msg->angular.z = 0;
				break;
			case 'a':
				cmd_msg->linear.x = 0;
				cmd_msg->angular.z = 1;
				break;
			case 'd':
				cmd_msg->linear.x = 0;
				cmd_msg->angular.z = -1;
				break;
			
			default:
				cmd_msg->linear.x = 0;
				cmd_msg->angular.z = 0;
				break;
			}

			cmd_pub->publish(std::move(cmd_msg));
		
		}

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
		rclcpp::TimerBase::SharedPtr timer_key_read;
		char c;
    
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Teleop>());
	rclcpp::shutdown();
}
