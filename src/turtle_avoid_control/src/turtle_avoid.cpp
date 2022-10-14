// @toffanetto
// Exercício 3 -- 
// Autor: Gabriel Toffanetto França da Rocha
// Universidade Federal de Itajubá -- Campus Itabira
// ECAi21.2 -- Laboratório de Robótica Móvel
// 2022.2

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <assert.h>
#include <string>
#include <cmath>
#include <iostream>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

// Definição dos valores máximos de velocidade para o simulador turtlesim
#define LINEAR_SPEED_MAX 2.0
#define ANGULAR_SPEED_MAX 2.0

// Definição dos ganhos proporcionais para velocidade linear e angular
#define Kp_L 0.8
#define Kp_A 1
#define Kp_Ld 0.3
#define Kp_Ad 1.5

// Declaração do nó de teleoperação para o simulador turtlesim
class TurtleAvoid : public rclcpp::Node{

	public:

		TurtleAvoid() : Node("turtle_avoid"){	
			// Definição da qualidade de serviço para a comunicação do tópico		
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());


			// Iniciaçização do serviço de /spawn
			turtle_spawn = this->create_client<turtlesim::srv::Spawn>("spawn");

			// Declaração da mensagem
			auto new_turtle = std::make_shared<turtlesim::srv::Spawn::Request>();
			new_turtle->x = 1.0;
			new_turtle->y = 1.0;
			new_turtle->theta = 0.0;
			new_turtle->name = "turtle2";

			// Aguardando a disponiblidade do serviço de /spawn
			while (!turtle_spawn->wait_for_service(1s)) {
				RCLCPP_INFO(this->get_logger(), "Waiting for /spawn service");
			}

			// Chamada do servoço de /spawn
			auto result = turtle_spawn->async_send_request(new_turtle);

			// Instanciação do publisher do tópico cmd_vel, com mensagem do tipo Twist
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", default_qos);

			// Instanciação do subscriber do tópico /turtle1/pose, com a posição da turtle1
			pose_1_sub = this->create_subscription<turtlesim::msg::Pose>(
								"/turtle1/pose", default_qos, 
								std::bind(&TurtleAvoid::pose_1_callback, this, _1));


			// Instanciação do subscriber do tópico /turtle2/pose, com a posição da turtle2
			pose_2_sub = this->create_subscription<turtlesim::msg::Pose>(
								"/turtle2/pose", default_qos, 
								std::bind(&TurtleAvoid::pose_2_callback, this, _1));

			// Declaração do ponto de destino inicial da turtle
			auto turtle2_target_ptr = std::make_shared<turtlesim::msg::Pose>();
			turtle2_target_ptr->x = 8;
			turtle2_target_ptr->y = 10;
			turtle2_target = turtle2_target_ptr;

			RCLCPP_INFO(this->get_logger(), "ECAi21 | Two Turtle");
		}

		~TurtleAvoid(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}

	private:
		// Declaração da função de callback do subscriber de posição da turtle1
		void pose_1_callback(const turtlesim::msg::Pose::SharedPtr target1);

		// Declaração da função de callback do subscriber de posição da turtle2
		void pose_2_callback(const turtlesim::msg::Pose::SharedPtr target2);

		// Declaração da função de comando da turtle2
		void follow_turtle();

		// Declaração da função de exibição da posição da turtle1 e turtle2 no terminal
		void show_pose();

		// Declaração da função de cálculo do erro de distância linear entre as turtle's
		double linear_error(const turtlesim::msg::Pose::SharedPtr target1, const turtlesim::msg::Pose::SharedPtr target2);

		// Declaração da função de cálculo do erro de ângulo entre as turtle's
		double yaw_error(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2);


		// Declaração da função de cálculo do erro de ângulo de repulsão entre as turtle's
		double yaw_error_avoid(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2);

		// Declaração da função de cálculo de módulo de vetor 2D
		double modulo(const turtlesim::msg::Pose::SharedPtr target);

		// Declaração da função de cálculo de produto interno entre vetores 2D
		double produto_interno(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2);

		// Declaração do publisher
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

		// Declaração do subscriber da turtle1
		rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_1_sub;

		// Declaração do subscriber da turtle2
		rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_2_sub;

		// Declaração do serviço /spawn
		rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr turtle_spawn;

		// Declaração da posição da turtle1
		turtlesim::msg::Pose::SharedPtr turtle1_pose;

		// Declaração da posição da turtle2
		turtlesim::msg::Pose::SharedPtr turtle2_pose;

		// Declaração da posição alvo da turtle2
		turtlesim::msg::Pose::SharedPtr turtle2_target;
		// Flags de recebimento dos tópicos
		bool pose_1_sub_ok = false;
		bool pose_2_sub_ok = false;
    int i = 0;
};

void TurtleAvoid::show_pose(){
	std::cout << "|++++++++++Turtle 2+++++++++|" << std::endl;
	std::cout << "|    x: " << turtle2_pose->x << std::endl;
	std::cout << "|    y: " << turtle2_pose->y << std::endl;
	std::cout << "|theta: " << turtle2_pose->theta << std::endl;
	std::cout << "|+++++++++++++++++++++++++++|" << std::endl << std::endl;
	std::cout << "|+++++++++++Target++++++++++|" << std::endl;
	std::cout << "|    x: " << turtle2_target->x << std::endl;
	std::cout << "|    y: " << turtle2_target->y << std::endl;
	std::cout << "|theta: " << turtle2_target->theta << std::endl;
	std::cout << "|+++++++++++++++++++++++++++|" << std::endl << std::endl;
}

// Cálculo da distância linear entre as duas turtle's
double TurtleAvoid::linear_error(const turtlesim::msg::Pose::SharedPtr target1, const turtlesim::msg::Pose::SharedPtr target2){
    return sqrt((target1->x - target2->x)*(target1->x - target2->x) + (target1->y - target2->y)*(target1->y - target2->y));
}

// Calculo do módulo de um vetor 2D
double TurtleAvoid::modulo(const turtlesim::msg::Pose::SharedPtr target){
    return sqrt((target->x)*(target->x) + (target->y )*(target->y));
}

// Calculo do produto interno de um vetor 2D
double TurtleAvoid::produto_interno(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2){
    return (target1->x)*(target2->x) + (target1->y)*(target2->y);
}

// Calculo do erro de ângulo entre as turtle's
double TurtleAvoid::yaw_error(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2){
	// Declaração da posição da turtle1 reduzida à posição da turtle2
    auto turtle_target_0 = std::make_shared<turtlesim::msg::Pose>();

	// Declaração do vetor unitário i
    auto x_ref = std::make_shared<turtlesim::msg::Pose>();
    x_ref->x = 1;
    x_ref->y = 0;

	// Redução da posição do alvo à turtle2
    turtle_target_0->x = target2->x - target1->x;
    turtle_target_0->y = target2->y - target1->y;

	// Cálculo do ângulo de yaw da turtle1 em relação ao plano reduzido à turtle2
    double yaw_target = (modulo(turtle_target_0)==0) ? 0 : copysign(acos(produto_interno(x_ref,turtle_target_0)/modulo(turtle_target_0)),turtle_target_0->y);

	// Remapeamento do ângulo de yaw entre 0 e 360°
    yaw_target = (yaw_target < 0) ? yaw_target+2*M_PI : yaw_target;

	// Remapeamento do ângulo theta da turtle2 entre 0 e 360°
    double theta = (target1->theta < 0) ? target1->theta+2*M_PI : target1->theta;

	// Obtenção do erro de angulo em módulo
    double error = abs(yaw_target - theta);

	// Adição do sinal de erro baseado na menor distância angular
	error = ((yaw_target > theta && error > M_PI) || (yaw_target < theta && error < M_PI)) ? -error : error;

    return error;
}

// Calculo do erro de ângulo entre as turtle's
double TurtleAvoid::yaw_error_avoid(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2){
	// Declaração da posição da turtle1 reduzida à posição da turtle2
    auto turtle_target_0 = std::make_shared<turtlesim::msg::Pose>();

	// Declaração do vetor unitário i
    auto x_ref = std::make_shared<turtlesim::msg::Pose>();
    x_ref->x = 1;
    x_ref->y = 0;

	// Redução da posição do alvo à turtle2
    turtle_target_0->x = target2->x - target1->x;
    turtle_target_0->y = target2->y - target1->y;

	// Cálculo do ângulo de yaw da turtle1 em relação ao plano reduzido à turtle2
    double yaw_target = (modulo(turtle_target_0)==0) ? 0 : copysign(acos(produto_interno(x_ref,turtle_target_0)/modulo(turtle_target_0)),turtle_target_0->y);

	// Remapeamento do ângulo de yaw entre 0 e 360°
    yaw_target = (yaw_target < 0) ? yaw_target+2*M_PI : yaw_target;

	// Adicionando M_PI para reverter o angulo
	yaw_target = (yaw_target+M_PI > 2*M_PI) ? (yaw_target+M_PI - 2*M_PI) : (yaw_target+M_PI );

	// Remapeamento do ângulo theta da turtle2 entre 0 e 360°
    double theta = (target1->theta < 0) ? target1->theta+2*M_PI : target1->theta;

	// Obtenção do erro de angulo em módulo
    double error = abs(yaw_target - theta);

	// Adição do sinal de erro baseado na menor distância angular
	error = ((yaw_target > theta && error > M_PI) || (yaw_target < theta && error < M_PI)) ? -error : error;

    return error;
}

// Callback de recebimento da posição da turtle1
void TurtleAvoid::pose_1_callback(const turtlesim::msg::Pose::SharedPtr target1){
	// Flag para acusar recebimento da posição da turtle1
	pose_1_sub_ok = true;
	turtle1_pose = target1;

	// Caso as flags acusem o recebimento das duas posições, é chamada a função de comando da turtle2
	if(pose_1_sub_ok && pose_2_sub_ok)
		this->follow_turtle();
}

// Callback de recebimento da posição da turtle2
void TurtleAvoid::pose_2_callback(const turtlesim::msg::Pose::SharedPtr target2){
	// Flag para acusar recebimento da posição da turtle2
	pose_2_sub_ok = true;
	turtle2_pose = target2;

	// Caso as flags acusem o recebimento das duas posições, é chamada a função de comando da turtle2
	if(pose_1_sub_ok && pose_2_sub_ok)
		this->follow_turtle();
}

void TurtleAvoid::follow_turtle(){
	// Reseta as flags
	pose_1_sub_ok = pose_2_sub_ok = false;

	// Declaração da mensagem do publisher e das velocidades lineares e angulares
	auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
	double linear_speed = 0;
	double angular_speed = 0;

	// Verificação se a turtle2 está no raio de obstáculo da turtle1
	if(this->linear_error(turtle2_pose,turtle1_pose) < 1.5){
		
		// Velocidade linear setada em função da proximidade entre as duas
		linear_speed = this->linear_error(turtle2_pose,turtle1_pose)*Kp_Ld;

		// Velocidade angular setada em função do erro entre o erro de direção entre as
		// duas tartarugas, somando M_PI, de forma à enviar a turtle2 para a direção oposta
		// ao obstáculo
		angular_speed = (this->yaw_error_avoid(turtle2_pose,turtle1_pose))*Kp_Ad;
		
		RCLCPP_WARN(this->get_logger(), "CLOSE TO TURTLE, avoiding...");
  	}

	// Caso não haja obstáculo, segue a navegação normalmente até o target point
  	else if(this->linear_error(turtle2_pose,turtle2_target) > 0.1){
		linear_speed = this->linear_error(turtle2_pose,turtle2_target)*Kp_L;
		angular_speed = this->yaw_error(turtle2_pose,turtle2_target)*Kp_A;
	}

	// Caso esteja no raio de convergência do target point, é sorteado um novo alvo no plano
	else{
		auto turtle2_target_ptr = std::make_shared<turtlesim::msg::Pose>();
		turtle2_target_ptr->x = rand() % 10 + 1;
		turtle2_target_ptr->y = rand() % 10 + 1;
		turtle2_target = turtle2_target_ptr;
	}

	// Testa se a turtle2 está muito próxima das paredes, caso sim, reduz sua velocidade linear
	// de forma proporcional, reduzindo o raio do ICR e desviando da parede
	if(turtle2_pose->x < 1){
		linear_speed = turtle2_pose->x*Kp_Ld;
		RCLCPP_WARN(this->get_logger(), "CLOSE TO WALL, avoiding...");
	}
	else if(turtle2_pose->x > 11){
		linear_speed = (12-turtle2_pose->x)*Kp_Ld;
		RCLCPP_WARN(this->get_logger(), "CLOSE TO WALL, avoiding...");
	}
	else if(turtle2_pose->y < 1){
		linear_speed = turtle2_pose->y*Kp_Ld;
		RCLCPP_WARN(this->get_logger(), "CLOSE TO WALL, avoiding...");
	}
	else if(turtle2_pose->y > 11){
		linear_speed = (12-turtle2_pose->y)*Kp_Ld;
		RCLCPP_WARN(this->get_logger(), "CLOSE TO WALL, avoiding...");
	}

	// Saturação das velocidades entre -LINEAR_SPEED_MAX e LINEAR_SPEED_MAX e entre -ANGULAR_SPEED_MAX e ANGULAR_SPEED_MAX
	// Atribuição das velocidades na mensagem
	cmd_msg->linear.x = (abs(linear_speed) > LINEAR_SPEED_MAX) ? copysign(LINEAR_SPEED_MAX,linear_speed) : linear_speed;
	cmd_msg->angular.z = (abs(angular_speed) > ANGULAR_SPEED_MAX) ? copysign(ANGULAR_SPEED_MAX,angular_speed) : angular_speed;

	// Publicação das velocidades para a turtle2 com base na lei de controle implementada
	cmd_pub->publish(std::move(cmd_msg));

	show_pose();
	
}

int main(int argc, char ** argv){
	
	// Inicialização do ROS2
	rclcpp::init(argc, argv);

	// Inicialização do nó em um executor de forma ciclica
	rclcpp::spin(std::make_shared<TurtleAvoid>());

	// Encerramento nó
	rclcpp::shutdown();
}
