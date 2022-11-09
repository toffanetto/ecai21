#include <cstdio>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <functional>
#include <queue>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;


#define LINEAR_SPEED_MAX 2.0
#define ANGULAR_SPEED_MAX 2.0
#define Kp_L 0.2
#define Kp_Ld 0.1
#define Kp_A 0.5
#define Kp_At 0.5
#define Kp_Ad 0.2

#define RANGE_CRASH 3
#define RANGE_SCANNER 10
#define ROBOT_WIDTH 0.84
#define WALL_DISTANCE 1


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

struct Descontinuidade{
	Ponto ponto;
	double d;
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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//                     Tanget Bug Control node class                            //
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

class TangentBugControl : public rclcpp::Node{

	public:

		TangentBugControl() : Node("tangent_bug_control"){
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

			laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/dolly/laser_scan", default_qos, std::bind(&TangentBugControl::laser_callback, this, _1));
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/dolly/cmd_vel", default_qos);
			cmd_timer = this->create_wall_timer(50ms, std::bind(&TangentBugControl::cmd_timer_callback,this));
            
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/dolly/odom", default_qos, std::bind(&TangentBugControl::odom_callback, this, _1));

		}

		~TangentBugControl(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}



	private:

		// Callback para receber os dados do laser
		void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

		// Callback para receber os dados da odometria
		void odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose);

		// Callback para processar os dados de navegação periodicamente
		void cmd_timer_callback();

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

		rclcpp::TimerBase::SharedPtr cmd_timer;

		sensor_msgs::msg::LaserScan::SharedPtr laser_data;
        nav_msgs::msg::Odometry::SharedPtr odom_data;

		bool laser_data_recived = false;
		bool odom_data_recived = false;

		bool wall_follow = false;
		bool O_target = false;
		bool goal_target = true;

		Ponto target = {1,1,0};
		Ponto robot_point;
		Euler robot_orientation;

};


void TangentBugControl::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
	laser_data = scan;
	laser_data_recived = true;
}

void TangentBugControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose){ 	
	robot_point = {pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z};
	Quaternion robot_quaternion = {pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w};
	robot_orientation = quaternion2euler(robot_quaternion);
	odom_data_recived = true;
}

void TangentBugControl::cmd_timer_callback(){
	// Processar laserscan e obter linear_speed e angular_speed


	auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
	double linear_speed = 0;
	double angular_speed = 0;

	// Executa o processamento do controle se tiverem chegado novas mensagens do laser
	if(laser_data_recived){ //&& odom_data_recived){

		std::pair<double,double> nearest_point, first_point, last_point;            
	
		bool first=true, crash = false;

		double theta, phi, error_dw = 0;
		int near_index = 0;
		
		auto cmp = [](Descontinuidade left, Descontinuidade right) { return (left.d) > (right.d); };

		std::priority_queue<Descontinuidade, std::vector<Descontinuidade>, decltype(cmp)> desc(cmp);

		// Processamento dos dados do vetor de pontos do laser
		for(auto i=0u; i<laser_data->ranges.size();i++){

			// Obtém range do raio atual
			auto range = laser_data->ranges.at(i);
			
			// Calcula ângulo do raio atual
			theta = laser_data->angle_increment*i + laser_data->angle_min;

			// Caso o range medido for menor que o raio de detecção de objeto estipulado
			if(range <= RANGE_CRASH){
				
				// Armazena a primeira posição do obstáculo
				if(first){ // Inicio da detecção de obstáculo
					first_point = std::make_pair(range, theta);
					nearest_point = first_point;
					last_point = first_point;
					near_index = i;
				}

				// Armazena o ponto mais próximo do obstáculo
				if(range < nearest_point.first){ // Definição do ponto mais próximo do robô
					nearest_point = std::make_pair(range, theta);
					near_index = i;
				}

				// Armaena o ultimo ponto do obstáculo
				last_point = std::make_pair(range, theta); // Definição do último ponto conhecido do obstáculo

				// Calcula se o obstáculo está no trajeto que o robô está fazendo
				if(range*cos(theta) <= RANGE_CRASH && range*abs(sin(theta)) < ROBOT_WIDTH/2){ // Validação se o robô irá bater no obstáculo que ele encontrou
					crash = true;
				}

			}

			if(i >= 1){
				if(abs(laser_data->ranges.at(i)-laser_data->ranges.at(i-1))>1){
					Descontinuidade Oi;
					Oi.ponto.x = laser_data->ranges.at(i-1)*cos((i-1)*laser_data->angle_increment + laser_data->angle_min);
					Oi.ponto.y = laser_data->ranges.at(i-1)*sin((i-1)*laser_data->angle_increment + laser_data->angle_min);
					Oi.d = distPoints(Oi.ponto,target) + distPoints(robot_point,target);

					desc.push(Oi);
				}
			}

			
		}

		if(crash && goal_target){
			O_target = true;
			goal_target = false;
		}

		// Seguir parede
		if(wall_follow){
			RCLCPP_INFO(this->get_logger(), "Wall following.");

			if(nearest_point.second == last_point.second && near_index >= 1){
				
				double x1,y1,x2,y2,xr,yr;

				x1 = laser_data->ranges.at(near_index)*cos(near_index*laser_data->angle_increment + laser_data->angle_min);
				y1 = laser_data->ranges.at(near_index)*sin(near_index*laser_data->angle_increment + laser_data->angle_min);
				x2 = laser_data->ranges.at(near_index-1)*cos((near_index-1)*laser_data->angle_increment + laser_data->angle_min);
				y2 = laser_data->ranges.at(near_index-1)*sin((near_index-1)*laser_data->angle_increment + laser_data->angle_min);

				xr = x2-x1;
				yr = y2-y1;

				Ponto Vr0 = {xr,yr,0};

				phi = getYaw(Vr0);
			}
			else{
				double x1,y1,x2,y2,xr,yr;

				x1 = laser_data->ranges.at(near_index)*cos(near_index*laser_data->angle_increment + laser_data->angle_min);
				y1 = laser_data->ranges.at(near_index)*sin(near_index*laser_data->angle_increment + laser_data->angle_min);
				x2 = laser_data->ranges.at(near_index+1)*cos((near_index+1)*laser_data->angle_increment + laser_data->angle_min);
				y2 = laser_data->ranges.at(near_index+1)*sin((near_index+1)*laser_data->angle_increment + laser_data->angle_min);

				xr = x2-x1;
				yr = y2-y1;

				Ponto Vr0 = {xr,yr,0};

				phi = getYaw(Vr0);
			}

			if(nearest_point.first > WALL_DISTANCE){
				error_dw = nearest_point.first-WALL_DISTANCE;
			}
			else if(nearest_point.first < WALL_DISTANCE){
				error_dw = nearest_point.first-WALL_DISTANCE;
			}

			if(near_index < int(laser_data->ranges.size())/2){
				phi -= M_PI;
				error_dw *= -1;
			}

			// Obtenção das velelocidades lineares e angulares por meio da lei de controle implementada
			linear_speed = ((abs(Kp_Ld*phi) - abs(Kp_Ld*error_dw)) < LINEAR_SPEED_MAX*Kp_L) ? LINEAR_SPEED_MAX*Kp_L - abs(Kp_Ld*phi) - abs(Kp_Ld*error_dw) : 0;
			angular_speed = ((phi*Kp_At + error_dw*Kp_Ad) < ANGULAR_SPEED_MAX) ? phi*Kp_At + error_dw*Kp_Ad : copysign(ANGULAR_SPEED_MAX,phi);
		}

		// Navegação em linha reta para o destino (m-line)
		if(goal_target){
			RCLCPP_INFO(this->get_logger(), "Navigate to goal.");

			double error_linear = distPoints(robot_point,target);
			double error_angular = abs(getYaw(target)-robot_orientation.yaw); 

			error_angular = ((getYaw(target) > robot_orientation.yaw && error_angular > M_PI) || (robot_orientation.yaw < getYaw(target) && error_angular < M_PI)) ? -error_angular : error_angular;

			// Obtenção das velelocidades lineares e angulares por meio da lei de controle implementada
			linear_speed = error_linear*Kp_L - error_angular*Kp_Ld;
			angular_speed = error_angular*Kp_A;

			// Saturadores
			if(linear_speed > LINEAR_SPEED_MAX)
				linear_speed = LINEAR_SPEED_MAX;
			else if(linear_speed < 0)
				linear_speed = 0;

			if(abs(angular_speed) > ANGULAR_SPEED_MAX)
				angular_speed = copysign(ANGULAR_SPEED_MAX,angular_speed);
		}

		// Navegação para uma descontinuidade
		if(O_target){
			RCLCPP_INFO(this->get_logger(), "Navigate to discontinuity.");

			Descontinuidade O = desc.top();

			double error_linear = distPoints(robot_point,O.ponto);
			double error_angular = abs(getYaw(O.ponto)-robot_orientation.yaw); 

			error_angular = ((getYaw(O.ponto) > robot_orientation.yaw && error_angular > M_PI) || (robot_orientation.yaw < getYaw(O.ponto) && error_angular < M_PI)) ? -error_angular : error_angular;

			// Obtenção das velelocidades lineares e angulares por meio da lei de controle implementada
			linear_speed = error_linear*Kp_L - error_angular*Kp_Ld;
			angular_speed = error_angular*Kp_A;

			// Saturadores
			if(linear_speed > LINEAR_SPEED_MAX)
				linear_speed = LINEAR_SPEED_MAX;
			else if(linear_speed < 0)
				linear_speed = 0;

			if(abs(angular_speed) > ANGULAR_SPEED_MAX)
				angular_speed = copysign(ANGULAR_SPEED_MAX,angular_speed);
		}


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
	rclcpp::spin(std::make_shared<TangentBugControl>());

	// Encerramento nó
	rclcpp::shutdown();

	return 0;
}
