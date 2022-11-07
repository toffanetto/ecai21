#include <cstdio>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;


#define LINEAR_SPEED_MAX 2.0
#define ANGULAR_SPEED_MAX 2.0
#define Kp_L 0.2
#define Kp_Ld 0.1
#define Kp_A 0.7

#define RANGE_CRASH 3
#define ROBOT_WIDTH 0.84

class ReactiveControl : public rclcpp::Node{

	public:

		ReactiveControl() : Node("reactive_control"){
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

			laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/dolly/laser_scan", default_qos, std::bind(&ReactiveControl::laser_callback, this, _1));
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/dolly/cmd_vel", default_qos);
			cmd_timer = this->create_wall_timer(50ms, std::bind(&ReactiveControl::cmd_timer_callback,this));

		}

		~ReactiveControl(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}



	private:

		// Callback para receber os dados do laser
		void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

		// Callback para processar os dados de navegação periodicamente
		void cmd_timer_callback();

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
		rclcpp::TimerBase::SharedPtr cmd_timer;
		sensor_msgs::msg::LaserScan::SharedPtr laser_data;

		bool laser_data_recived = false;

};


void ReactiveControl::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
	laser_data = scan;
	laser_data_recived = true;
}

void ReactiveControl::cmd_timer_callback(){
	// Processar laserscan e obter linear_speed e angular_speed


	auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
	double linear_speed = 0;
	double angular_speed = 0;

	// 1 -- Desviar de obstáculo
	// 		a -- Desviando para esquerda
	// 		b -- Desviando para direita

	// Executa o processamento do controle se tiverem chegado novas mensagens do laser
	if(laser_data_recived){

		std::pair<double,double> nearest_point, first_point, last_point;            
	
		bool first=true, crash=false;

		double alpha, range_max, theta, phi;
		
		int turn = 1;

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
				}

				// Armazena o ponto mais próximo do obstáculo
				if(range < nearest_point.first) // Definição do ponto mais próximo do robô
					nearest_point = std::make_pair(range, theta);

				// Armaena o ultimo ponto do obstáculo
				last_point = std::make_pair(range, theta); // Definição do último ponto conhecido do obstáculo
			}

			// Calcula se o obstáculo está no trajeto que o robô está fazendo
			if(range*cos(theta) <= RANGE_CRASH && range*abs(sin(theta)) < ROBOT_WIDTH/2){ // Validação se o robô irá bater no obstáculo que ele encontrou
				crash = true;
			}

		}
		
		// Caso haja obstáculo do trajeto do robô
		if(crash){

			// Verifica para qual lado é melhor virar com base no menor ângulo entre o robô e os pontos de início e fim do obstáculo
			if(abs(first_point.second)>abs(last_point.second)){
				range_max = last_point.first;
				alpha = last_point.second;
				turn = 1;
				RCLCPP_INFO(this->get_logger(), "Obstacle avoiding by left.");
			}
			else{
				range_max = first_point.first;
				alpha = first_point.first;
				turn = -1;
				RCLCPP_INFO(this->get_logger(), "Obstacle avoiding by rigth.");
			}

			// Obtenção do ângulo de correção de tranjetória em relação a extremidade do obstáculo1
			double cos_theta = ((ROBOT_WIDTH*ROBOT_WIDTH - 2*range_max*range_max)/(- 2*range_max*range_max));

			theta = (abs(cos_theta) > 1) ? copysign(M_PI/2,turn) : copysign(acos(cos_theta),turn);

			phi = theta/2 + alpha; // Erro de angulo para sair do obstáculo
		}
		else
			RCLCPP_INFO(this->get_logger(), "Normal navigation.");

		// Obtenção das velelocidades lineares e angulares por meio da lei de controle implementada
		linear_speed = (Kp_Ld*phi < LINEAR_SPEED_MAX*Kp_L) ? LINEAR_SPEED_MAX*Kp_L - Kp_Ld*phi : 0;
		angular_speed = copysign(phi,turn)*Kp_A;


		cmd_msg->linear.x = (abs(linear_speed) > LINEAR_SPEED_MAX) ? copysign(LINEAR_SPEED_MAX,linear_speed) : linear_speed;
		cmd_msg->angular.z = (abs(angular_speed) > ANGULAR_SPEED_MAX) ? copysign(ANGULAR_SPEED_MAX,angular_speed) : angular_speed;

		cmd_pub->publish(std::move(cmd_msg));

		laser_data_recived = false;
	}
	else
		RCLCPP_WARN(this->get_logger(), "No data recived from LaserScan yet.");
}

int main(int argc, char ** argv){
	
	// Inicialização do ROS2
	rclcpp::init(argc, argv);

	// Inicialização do nó em um executor de forma ciclica
	rclcpp::spin(std::make_shared<ReactiveControl>());

	// Encerramento nó
	rclcpp::shutdown();

	return 0;
}
