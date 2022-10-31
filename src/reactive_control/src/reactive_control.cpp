#include <cstdio>
#include <cstdlib>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

#define LINEAR_SPEED_MAX 2.0
#define ANGULAR_SPEED_MAX 2.0
#define Kp_L 1
#define Kp_A 1

class reactiveControl : public rclcpp::Node{

	public:

		reactiveControl() : Node("reactive_control"){
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

			laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("dolly/scan", default_qos, std::bind(&reactiveControl::laser_callback, this, _1));
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("dolly/cmd_vel", default_qos);

		}

		~reactiveControl(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}



	private:

		void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

		void show_status();

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

};


void reactiveControl::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
	
}

int main(int argc, char ** argv){
	
	// Inicialização do ROS2
	rclcpp::init(argc, argv);

	// Inicialização do nó em um executor de forma ciclica
	rclcpp::spin(std::make_shared<reactiveControl>());

	// Encerramento nó
	rclcpp::shutdown();

	return 0;
}
