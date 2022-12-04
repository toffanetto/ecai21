// @toffanetto
// 						  Exercício 5 -- Tangent Bug
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


#define LINEAR_SPEED_MAX 1.0
#define ANGULAR_SPEED_MAX 1.0
#define Kp_L 0.1
#define Kp_Ld 0.3
#define Kp_Lw 0.8
#define Kp_A 0.8
#define Kp_At 0.5
#define Kp_Ad 0.2

#define RANGE_CRASH 3
#define RANGE_SCANNER 7
#define ROBOT_WIDTH 0.84
#define WALL_DISTANCE 1

#define TARGET_X 20
#define TARGET_Y 0


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
	bool inicio = true;
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
		bool no_solution = false;
		bool wall_follow_init = true;

		Ponto O_follow;

		std::vector<Descontinuidade> desc;

		Ponto target = {TARGET_X,TARGET_Y,0};
		Ponto robot_point;
		Euler robot_orientation;

};


void TangentBugControl::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
	laser_data = scan;
	laser_data_recived = true;
}

void TangentBugControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose){ 	
	robot_point = {pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z};
	Quaternion robot_quaternion = {pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z};
	robot_orientation = quaternion2euler(robot_quaternion);
	odom_data_recived = true;

}

void TangentBugControl::cmd_timer_callback(){
	// Processar laserscan e obter linear_speed e angular_speed


	auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
	double linear_speed = 0;
	double angular_speed = 0;

	// Executa o processamento do controle se tiverem chegado novas mensagens do laser
	if(laser_data_recived && odom_data_recived){

		std::pair<double,double> nearest_point, first_point, last_point;            
	
		bool first=true, crash = false, obstacle = false;

		double theta = 0, phi = 0, error_dw = 0;
		int near_index = 0;

		if(!O_target)
			desc.clear();

		// Processamento dos dados do vetor de pontos do laser
		for(auto i=0u; i<laser_data->ranges.size();i++){

			// Obtém range do raio atual
			auto range = laser_data->ranges.at(i);
			
			// Calcula ângulo do raio atual
			theta = laser_data->angle_increment*i + laser_data->angle_min;

			// Caso o range medido for menor que o raio de detecção de objeto estipulado
			if(range <= RANGE_CRASH){

				obstacle = true;
				
				// Armazena a primeira posição do obstáculo
				if(first){ // Inicio da detecção de obstáculo
					first_point = std::make_pair(range, theta);
					nearest_point = first_point;
					last_point = first_point;
					near_index = i;
					first = false;
				}

				// Armazena o ponto mais próximo do obstáculo
				if(range < nearest_point.first && range != std::numeric_limits<double>::infinity()){ // Definição do ponto mais próximo do robô
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

			if(i >= 1 && goal_target){

				if(abs(laser_data->ranges.at(i)-laser_data->ranges.at(i-1))>1){ // Ocorreu uma descontinuidade
					
					Descontinuidade Oi;

					double range1 = (laser_data->ranges.at(i-1) == std::numeric_limits<double>::infinity()) ? 10 : laser_data->ranges.at(i-1);
					double range2 = (laser_data->ranges.at(i) == std::numeric_limits<double>::infinity()) ? 10 : laser_data->ranges.at(i);

					Oi.ponto.x = (std::min(range1,range2))*cos((i-1)*laser_data->angle_increment + laser_data->angle_min) + robot_point.x;
					Oi.ponto.y = (std::min(range1,range2))*sin((i-1)*laser_data->angle_increment + laser_data->angle_min) + robot_point.y;

					Oi.d = distPoints(Oi.ponto,target) + distPoints(robot_point,target);

					Oi.ponto.x = (std::min(range1,range2)-1.5)*cos((i-1)*laser_data->angle_increment + laser_data->angle_min) + robot_point.x;
					Oi.ponto.y = (std::min(range1,range2)-1.5)*sin((i-1)*laser_data->angle_increment + laser_data->angle_min) + robot_point.y;

					Oi.inicio = (range2 < range1) ? true : false;

					
					if(desc.empty()){ // Se estiver vazia, adiciona a primeira
						desc.push_back(Oi);
					}
					else{
						if(range2 < range1){ // Se detectar o início de um obstáculo...
							if(desc.back().inicio){ // Se ja existia um início, tira e coloca o novo
								desc.pop_back();
								desc.push_back(Oi);
							}
							else{ // Se era um fim de obstáculo...
								if(distPoints(Oi.ponto,desc.back().ponto) > 2*ROBOT_WIDTH){  // Verifica se o robo pode passar no espaço entre eles
									desc.push_back(Oi); 
								}
								else{
									desc.pop_back(); // Se não puder, desconsidera o antigo fim.
								}
							}
						}
						else if(range2 > range1){// Se detectar o fim de um obstáculo...
							if(desc.back().inicio){ // Se existia um inicio, adiciona o fim do mesmo
								desc.push_back(Oi);
							}
							else{ // Se existia um fim, ele é removido e adiciona o fim novo
								desc.pop_back();
								desc.push_back(Oi);
							}
						}				
					}
				}
			}

			
		}

		
		auto cmp = [](Descontinuidade left, Descontinuidade right) { return (left.d) < (right.d); };

		std::sort(desc.begin(),desc.end(), cmp);

		// troca de contexto goal_target -> O_target
		if(crash && goal_target){
			O_target = true;
			goal_target = false;
		}

		// Seguir parede
		if(wall_follow){
			RCLCPP_INFO(this->get_logger(), "Wall following.");

			if(distPoints(robot_point, O_follow) > 0.6){
				wall_follow_init = false;
			}

			if(distPoints(robot_point, O_follow) < 0.5 && !wall_follow_init){
				wall_follow = false;
				no_solution = true;
			}

			if(obstacle){

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

				error_dw = (near_index < int(laser_data->ranges.size())/2) ?  -(nearest_point.first-WALL_DISTANCE) : (nearest_point.first-WALL_DISTANCE);

				phi = (phi > M_PI_2) ? phi-M_PI : phi;

			}


			// Obtenção das velelocidades lineares e angulares por meio da lei de controle implementada
			linear_speed = ((abs(Kp_Ld*phi) - abs(Kp_Ld*error_dw)) < LINEAR_SPEED_MAX*Kp_Lw) ? LINEAR_SPEED_MAX*Kp_Lw - abs(Kp_Ld*phi) - abs(Kp_Ld*error_dw) : 0;
			angular_speed = (abs(phi*Kp_At + error_dw*Kp_Ad) < ANGULAR_SPEED_MAX) ? phi*Kp_At + error_dw*Kp_Ad : copysign(ANGULAR_SPEED_MAX,phi);
			
			// Troca de contexto wall_follow -> goal_target

			if(abs(getYaw(robot_point,target)-robot_orientation.yaw) < 0.01 && laser_data->ranges.at(int(laser_data->ranges.size())/2) > RANGE_SCANNER && !crash){
					wall_follow = false;
					goal_target = true;
					wall_follow_init = true;
			}
		}

		// Navegação em linha reta para o destino (m-line)
		if(goal_target){

			double error_linear = distPoints(robot_point,target);
			double error_angular = (getYaw(robot_point,target)-robot_orientation.yaw); 

			// Obtenção das velelocidades lineares e angulares por meio da lei de controle implementada
			linear_speed = error_linear*Kp_L - error_angular*Kp_Ld;
			angular_speed = error_angular*Kp_A;

			if(error_linear < 0.5){
				linear_speed = 0;
				angular_speed = 0;
				RCLCPP_INFO(this->get_logger(), "Reached goal.");
			}
			else{
				RCLCPP_INFO(this->get_logger(), "Navigating to goal.");
			}

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
			RCLCPP_INFO(this->get_logger(), "Navigating to discontinuity.");

			if(desc.empty()){
				O_target = false;
				wall_follow = true;
			}
			else{
				Descontinuidade O = desc.front();

				double error_linear = distPoints(robot_point,O.ponto);
				double error_angular = (getYaw(robot_point,O.ponto)-robot_orientation.yaw); 

				// Obtenção das velelocidades lineares e angulares por meio da lei de controle implementada
				linear_speed = error_linear*Kp_L - error_angular*Kp_Ld + 0.2;
				angular_speed = error_angular*Kp_A;

				// Saturadores
				if(linear_speed > LINEAR_SPEED_MAX)
					linear_speed = LINEAR_SPEED_MAX;
				else if(linear_speed < 0)
					linear_speed = 0;

				if(abs(angular_speed) > ANGULAR_SPEED_MAX)
					angular_speed = copysign(ANGULAR_SPEED_MAX,angular_speed);

				// Troca de contexto O_target -> wall_follow
				if(error_linear < 0.2){
					O_target = false;
					wall_follow = true;
					O_follow = O.ponto;
				}
			}
		}

		if(no_solution){
			linear_speed = 0;
			angular_speed = 0;
			RCLCPP_ERROR(this->get_logger(), "Don't have a solution to reach the target.");
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
