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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

////////////////////////////////////////////////////////////////////////////////
// Constants

#define TARGET_X 40
#define TARGET_Y 0

#define LINEAR_SPEED_MAX 1.0
#define ANGULAR_SPEED_MAX 1.0
#define Kp_L 0.2
#define Kp_Ld 0.2
#define Kp_A 0.2

#define K_ATT 0.3
#define K_REP 1.2
#define P0 4

#define LIDAR_X 0.5
#define LIDAR_Y 0

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

////////////////////////////////////////////////////////////////////////////////
// Function to give the module of the vector
double modulo(Ponto p){
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

////////////////////////////////////////////////////////////////////////////////
// Function to give intern product of two vectors
double prodInter(Ponto v, Ponto u){
    return (v.x*u.x + v.y*u.y + v.z*u.z);
}

////////////////////////////////////////////////////////////////////////////////
// Function to distance into two points
double distPoints(Ponto x, Ponto y){
    return sqrt((x.x-y.x)*(x.x-y.x) + (x.y-y.y)*(x.y-y.y));
}

////////////////////////////////////////////////////////////////////////////////
// Function to give angle between x axys and destination point (radians) 
// using robot pose with origin reference
double getYaw(Ponto robot, Ponto point){
    Ponto u = {point.x - robot.x, point.y - robot.y, point.z - robot.z};
    Ponto x = {1,0,0};
    return (modulo(u)==0) ? 0 : copysign(acos(prodInter(x,u)/modulo(u)),u.y);
}

////////////////////////////////////////////////////////////////////////////////
// Function to give angle between x axys and destination point (radians) 
// using origin reference
double getYaw(Ponto point){
    Ponto u = {point.x, point.y, point.z};
    Ponto x = {1,0,0};
    return (modulo(u)==0) ? 0 : copysign(acos(prodInter(x,u)/modulo(u)),u.y);
}

////////////////////////////////////////////////////////////////////////////////
// Function to give the magnitude of a vector
double getMag(Ponto point){
	return sqrt(point.x*point.x + point.y*point.y);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//                     Potential Field Control node class                            //
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

class PotentialFieldControl : public rclcpp::Node{

	public:

		PotentialFieldControl() : Node("potential_field_control"){
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

			laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/dolly/laser_scan", default_qos, std::bind(&PotentialFieldControl::laser_callback, this, _1));
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/dolly/cmd_vel", default_qos);
			cmd_timer = this->create_wall_timer(50ms, std::bind(&PotentialFieldControl::cmd_timer_callback,this));
            
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/dolly/odom", default_qos, std::bind(&PotentialFieldControl::odom_callback, this, _1));

		}

		~PotentialFieldControl(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}



	private:

		// Callback para receber os dados do laser
		void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

		// Callback para receber os dados da odometria
		void odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose);

		// Callback para processar os dados de navegação periodicamente
		void cmd_timer_callback();

		void getPose(double range, double theta, Ponto &obstacle);

		void getFr(Ponto obstacle, Ponto &Fr);

		void getFatt(Ponto goal, Ponto &Fatt);

		void getFres(std::vector<Ponto> F, Ponto &Fres);

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

		rclcpp::TimerBase::SharedPtr cmd_timer;

		sensor_msgs::msg::LaserScan::SharedPtr laser_data;
        nav_msgs::msg::Odometry::SharedPtr odom_data;

		bool laser_data_recived = false;
		bool odom_data_recived = false;

		Ponto target = {TARGET_X,TARGET_Y,0};
		Ponto robot_point;
		Euler robot_orientation;
};

////////////////////////////////////////////////////////////////////////////////
// LiDAR callback
void PotentialFieldControl::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
	laser_data = scan;
	laser_data_recived = true;
}

////////////////////////////////////////////////////////////////////////////////
// Odometry callback
void PotentialFieldControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose){ 	
	robot_point = {pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z};
	Quaternion robot_quaternion = {pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z};
	robot_orientation = quaternion2euler(robot_quaternion);
	odom_data_recived = true;
}

////////////////////////////////////////////////////////////////////////////////
// Method to get the pose of a obstacle by LiDAR data
void PotentialFieldControl::getPose(double range, double theta, Ponto &obstacle){
	// Referencial do robô
	double xr = range*cos(theta);
	double yr = range*sin(theta);

	// Referencial global
	obstacle.x = (xr*cos(robot_orientation.yaw) - yr*sin(robot_orientation.yaw)) + robot_point.x + LIDAR_X;
	obstacle.y = (xr*sin(robot_orientation.yaw) + yr*cos(robot_orientation.yaw)) + robot_point.y + LIDAR_Y;

	std::cout 	<< "Obstacle:  x = " << obstacle.x << std::endl
				<< "           y = " << obstacle.y << std::endl
				<< "       theta = " << theta << std::endl
				<< "       range = " << range << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Method to get repulsive force vector
void PotentialFieldControl::getFr(Ponto obstacle, Ponto &Fr){ // y = 2
	double p_i = distPoints(obstacle,robot_point);
	Fr.x = K_REP*1/(p_i*p_i*p_i)*(1/(p_i) - 1/(P0))*(robot_point.x - obstacle.x);
	Fr.y = K_REP*1/(p_i*p_i*p_i)*(1/(p_i) - 1/(P0))*(robot_point.y - obstacle.y);
}

////////////////////////////////////////////////////////////////////////////////
// Method to get atractive force vector
void PotentialFieldControl::getFatt(Ponto goal, Ponto &Fatt){ // Cálculo da força de atração utilizando campo parabólico
	Fatt.x = (goal.x - robot_point.x)*K_ATT;
	Fatt.y = (goal.y - robot_point.y)*K_ATT;
}

////////////////////////////////////////////////////////////////////////////////
// Method to get resultant force vector
void PotentialFieldControl::getFres(std::vector<Ponto> F, Ponto &Fres){
	Fres.x = 0;
	Fres.y = 0;
	
	for(auto Fi : F){
		Fres.x += Fi.x;
		Fres.y += Fi.y;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Timer callback to process LiDAR and odometry data to get linear and angular speed 
void PotentialFieldControl::cmd_timer_callback(){
	
	std::cout << "|+++++++++++++++++++++++++++++++++|" << std::endl;


	auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
	double linear_speed = 0;
	double angular_speed = 0;

	std::vector<Ponto> F;

	// Executa o processamento do controle se tiverem chegado novas mensagens do laser
	if(laser_data_recived && odom_data_recived){

		Ponto Fatt;

		getFatt(target,Fatt);

		F.push_back(Fatt);
		     
		// Processamento dos dados do vetor de pontos do laser
		for(auto i=0u; i<laser_data->ranges.size();i++){

			// Obtém range do raio atual
			auto range = laser_data->ranges.at(i);
			
			// Calcula ângulo do raio atual
			auto theta = laser_data->angle_increment*i + laser_data->angle_min;

			if(range <= P0){
				Ponto obstacle;
				getPose(range,theta,obstacle);
				Ponto Fr;
				getFr(obstacle,Fr);
				F.push_back(Fr);
			}
		}

		Ponto Fres;

		getFres(F,Fres);


		double Fres_mag = getMag(Fres);
		double Fres_ang = getYaw(Fres);
		double angle_error = Fres_ang - robot_orientation.yaw;

		linear_speed = Fres_mag*Kp_L - abs(angle_error*Kp_Ld);
		angular_speed = angle_error*Kp_A;

		if(distPoints(robot_point,target) < 0.2){
			linear_speed = 0;
			angular_speed = 0;
			RCLCPP_INFO(this->get_logger(), "Reached the destination.");
		}

		std::cout << "Fatt: 	 x = " << Fatt.x << std::endl
				  << "      	 y = " << Fatt.y << std::endl
				  << "Fres:      x = " << Fres.x << std::endl
				  << "      	 y = " << Fres.y << std::endl
				  << "Mov:  linear = " << linear_speed << std::endl
				  << "     angular = " << angular_speed << std::endl
				  << "Pose:      x = " << robot_point.x << std::endl
				  << "           y = " << robot_point.y << std::endl
				  << "         yaw = " << robot_orientation.yaw << std::endl;

		// Saturadores
		if(linear_speed > LINEAR_SPEED_MAX)
			linear_speed = LINEAR_SPEED_MAX;
		else if(linear_speed < 0)
			linear_speed = 0;

		if(abs(angular_speed) > ANGULAR_SPEED_MAX)
			angular_speed = copysign(ANGULAR_SPEED_MAX,angular_speed);

		cmd_msg->linear.x = (abs(linear_speed) > LINEAR_SPEED_MAX) ? copysign(LINEAR_SPEED_MAX,linear_speed) : linear_speed;
		cmd_msg->angular.z = (abs(angular_speed) > ANGULAR_SPEED_MAX) ? copysign(ANGULAR_SPEED_MAX,angular_speed) : angular_speed;

		cmd_pub->publish(std::move(cmd_msg));

		laser_data_recived = false;
		odom_data_recived = false;
	}
	else{
		if(!laser_data_recived)
			RCLCPP_WARN(this->get_logger(), "No data recived from LaserScan yet.");
		if(!odom_data_recived)
			RCLCPP_WARN(this->get_logger(), "No data recived from Odometry yet.");
	}
}

int main(int argc, char ** argv){
	
	// Inicialização do ROS2
	rclcpp::init(argc, argv);

	// Inicialização do nó em um executor de forma ciclica
	rclcpp::spin(std::make_shared<PotentialFieldControl>());

	// Encerramento nó
	rclcpp::shutdown();

	return 0;
}
