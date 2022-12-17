// @toffanetto
// 						  Exercício 6 -- Potential Field
// Autor: Gabriel Toffanetto França da Rocha
// Professor: André Chaves Magalhâes
// Federal University of Itajubá
// 2022.2
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <functional>
#include <algorithm>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

////////////////////////////////////////////////////////////////////////////////
// Constants

#define A 10 // Amplitude
#define B 10	// Velocidade

#define LINEAR_SPEED_MAX 2.0
#define ANGULAR_SPEED_MAX 2.0

#define K 0.3
#define D 0.2

#define X_R 0.5
#define Y_R 0.5

////////////////////////////////////////////////////////////////////////////////
// Structs and classes to new data types
struct Quaternion{
    double w,x,y,z;
};

struct Euler{
    double roll, pitch, yaw;
};

struct Ponto{
    double x, y, z;
};


////////////////////////////////////////////////////////////////////////////////
// Function to convert quaternion to euler angles (in radians)
Euler quaternion2euler(Quaternion q){
    Euler euler_angles;

    double x;
    double y;   

    x = 2*(q.w*q.x+q.y*q.z);
    y = 1 - 2*(q.x*q.x+q.y*q.y);
    euler_angles.roll = std::atan2(x,y); // sinal trocado

    x = 2*(q.w*q.y-q.z*q.x);
    euler_angles.pitch = (std::abs(x)>=1) ? copysign(M_PI/2,x) : asin(x);

    x = 2*(q.w*q.z+q.x*q.y);
    y = 1 - 2*(q.y*q.y+q.z*q.z);
    euler_angles.yaw = std::atan2(x,y);

    return euler_angles;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//                 Feedback Linearization Control node class                   //
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
class FeedbackLinearizationControl : public rclcpp::Node{

	public:

		FeedbackLinearizationControl() : Node("feedback_linearization_control"){
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/dolly/cmd_vel", default_qos);
			curve_pub = this->create_publisher<visualization_msgs::msg::Marker>("/dolly/curve", default_qos);
			cmd_timer = this->create_wall_timer(15ms, std::bind(&FeedbackLinearizationControl::cmd_timer_callback,this));
			odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/dolly/odom", default_qos, std::bind(&FeedbackLinearizationControl::odom_callback, this, _1));

			t_init = (now().nanoseconds())*1e-9;
		}

		~FeedbackLinearizationControl(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}

	private:

		// Callback para receber os dados da odometria
		void odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose);

		// Callback para processar os dados de navegação periodicamente
		void cmd_timer_callback();

		// Função que retorna o ponto atual da trajetória
		void getLemniscatePoint(double &x, double &y);

		// Função que computa a lei de controle do Feedback Linearization
		void getFeedbackLinearizationControl(double &v, double &w, double &x, double &y);
		
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr curve_pub;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

		rclcpp::TimerBase::SharedPtr cmd_timer;

        nav_msgs::msg::Odometry::SharedPtr odom_data;

		bool odom_data_recived = false;

		Ponto robot_point;
		Euler robot_orientation;

		double t_init = 0, x_old = 0, y_old = 0;

};

////////////////////////////////////////////////////////////////////////////////
// Odometry callback
void FeedbackLinearizationControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose){ 	
	robot_point = {pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z};
	Quaternion robot_quaternion = {pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z};
	robot_orientation = quaternion2euler(robot_quaternion);
	odom_data_recived = true;
}

////////////////////////////////////////////////////////////////////////////////
// Function to return the current point of the Lemniscate curve
void FeedbackLinearizationControl::getLemniscatePoint(double &x, double &y){
	double t = (now().nanoseconds())*1e-9 - t_init;
	x = (A*cos(t/B))/(1+sin(t/B)*sin(t/B));
	y = (A*sin(t/B)*cos(t/B))/(1+sin(t/B)*sin(t/B));
}


///////////////////////////////////////////////////////////////////////////////////
// Function compute the control action from the Feedback Linearization Control Law
void FeedbackLinearizationControl::getFeedbackLinearizationControl(double &v, double &w, double &x, double &y){
	double x_dot_r = 0.5, y_dot_r = 0.5;

	if(x_old == 0 && y_old == 0){
		x_old = x;
		y_old = y;
	}
	else{
		x_dot_r = (x - x_old)/20e-3;
		y_dot_r = (y - y_old)/20e-3;
		x_old = x;
		y_old = y;
	}

	double u1 = x_dot_r + K*(x - robot_point.x);
	double u2 = y_dot_r + K*(y - robot_point.y);

	v = cos(robot_orientation.yaw)*u1 + sin(robot_orientation.yaw)*u2;
	w = -sin(robot_orientation.yaw)/D*u1 + cos(robot_orientation.yaw)/D*u2;
}

////////////////////////////////////////////////////////////////////////////////
// Timer callback to process LiDAR and odometry data to get linear and angular speed 
void FeedbackLinearizationControl::cmd_timer_callback(){

	if(odom_data_recived){


	auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();

		double v, w, x, y;

		getLemniscatePoint(x,y);

		getFeedbackLinearizationControl(v,w,x,y);

		cmd_msg->linear.x = (abs(v) > LINEAR_SPEED_MAX) ? copysign(LINEAR_SPEED_MAX,v) : v;
		cmd_msg->angular.z = (abs(w) > ANGULAR_SPEED_MAX) ? copysign(ANGULAR_SPEED_MAX,w) : w;

		cmd_pub->publish(std::move(cmd_msg));

		std::cout << "++++++++++++++++"<< std::endl
				  << "X: " << x << std::endl
				  << "Y: " << y << std::endl
				  << "Xr: " << robot_point.x << std::endl
				  << "Yr: " << robot_point.y << std::endl
				  << "Xdotr: " << x_dot_r << std::endl
				  << "Ydotr: " << y_dot_r << std::endl
				  << "E : " << "(" << (-robot_point.x + x) << "," << (-robot_point.y + y) << ")" << std::endl
				  << "v: " << v<< std::endl
				  << "w: " << w << std::endl
				  << "theta: " << robot_orientation.yaw << std::endl
				  << "++++++++++++++++"<< std::endl;

	}
	else
		RCLCPP_WARN(this->get_logger(), "No data recived from Odometry yet.");
	odom_data_recived = false;

}

int main(int argc, char ** argv){
	
	// Inicialização do ROS2
	rclcpp::init(argc, argv);

	// Inicialização do nó em um executor de forma ciclica
	rclcpp::spin(std::make_shared<FeedbackLinearizationControl>());

	// Encerramento nó
	rclcpp::shutdown();

	return 0;
}
