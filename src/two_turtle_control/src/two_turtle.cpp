#include <cstdio>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <assert.h>
#include <string>
#include <cmath>
#include <iostream>

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
#define Kp_L 1
#define Kp_A 1

// Declaração do nó de teleoperação para o simulador turtlesim
class TwoTurtle : public rclcpp::Node{

	public:

		TwoTurtle() : Node("two_turtle"){	
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
								std::bind(&TwoTurtle::pose_1_callback, this, _1));


			// Instanciação do subscriber do tópico /turtle2/pose, com a posição da turtle2
			pose_2_sub = this->create_subscription<turtlesim::msg::Pose>(
								"/turtle2/pose", default_qos, 
								std::bind(&TwoTurtle::pose_2_callback, this, _1));

			RCLCPP_INFO(this->get_logger(), "ECAi21 | Two Turtle");
		}

		~TwoTurtle(){
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
		double linear_error();

		// Declaração da função de cálculo do erro de ângulo entre as turtle's
		double yaw_error();

		// Declaração da função de cálculo do erro de orientação entre as turtle's
		double orientation_error();

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

		// Declaração da posição da turtle1
		turtlesim::msg::Pose::SharedPtr turtle2_pose;

		// Flags de recebimento dos tópicos
		bool pose_1_sub_ok = false;
		bool pose_2_sub_ok = false;
};

void TwoTurtle::show_pose(){
	std::cout << "|++++++++++Turtle 1+++++++++|" << std::endl;
	std::cout << "|    x: " << turtle1_pose->x << std::endl;
	std::cout << "|    y: " << turtle1_pose->y << std::endl;
	std::cout << "|theta: " << turtle1_pose->theta << std::endl;
	std::cout << "|+++++++++++++++++++++++++++|" << std::endl << std::endl;
	std::cout << "|++++++++++Turtle 2+++++++++|" << std::endl;
	std::cout << "|    x: " << turtle2_pose->x << std::endl;
	std::cout << "|    y: " << turtle2_pose->y << std::endl;
	std::cout << "|theta: " << turtle2_pose->theta << std::endl;
	std::cout << "|+++++++++++++++++++++++++++|" << std::endl << std::endl;
}

// Cálculo da distância linear entre as duas turtle's
double TwoTurtle::linear_error(){
    return sqrt((turtle1_pose->x - turtle2_pose->x)*(turtle1_pose->x - turtle2_pose->x) + (turtle1_pose->y - turtle2_pose->y)*(turtle1_pose->y - turtle2_pose->y));
}

// Calculo do módulo de um vetor 2D
double TwoTurtle::modulo(const turtlesim::msg::Pose::SharedPtr target){
    return sqrt((target->x)*(target->x) + (target->y )*(target->y));
}

// Calculo do produto interno de um vetor 2D
double TwoTurtle::produto_interno(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2){
    return (target1->x)*(target2->x) + (target1->y)*(target2->y);
}

// Calculo do erro de ângulo entre as turtle's
double TwoTurtle::yaw_error(){
	// Declaração da posição da turtle1 reduzida à posição da turtle2
    auto turtle1_pose_0 = std::make_shared<turtlesim::msg::Pose>();

	// Declaração do vetor unitário i
    auto x_ref = std::make_shared<turtlesim::msg::Pose>();
    x_ref->x = 1;
    x_ref->y = 0;

	// Redução da posição da turtle1 à turtle2
    turtle1_pose_0->x = turtle1_pose->x - turtle2_pose->x;
    turtle1_pose_0->y = turtle1_pose->y - turtle2_pose->y;

	// Cálculo do ângulo de yaw da turtle1 em relação ao plano reduzido à turtle2
    double yaw_target = (modulo(turtle1_pose_0)==0) ? 0 : copysign(acos(produto_interno(x_ref,turtle1_pose_0)/modulo(turtle1_pose_0)),turtle1_pose_0->y);

	// Remapeamento do ângulo de yaw entre 0 e 360°
    yaw_target = (yaw_target < 0) ? yaw_target+2*M_PI : yaw_target;

	// Remapeamento do ângulo theta da turtle2 entre 0 e 360°
    double theta = (turtle2_pose->theta < 0) ? turtle2_pose->theta+2*M_PI : turtle2_pose->theta;

	// Obtenção do erro de angulo em módulo
    double error = abs(yaw_target - theta);

	// Adição do sinal de erro baseado na menor distância angular
	error = ((yaw_target > theta && error > M_PI) || (yaw_target < theta && error < M_PI)) ? -error : error;

    return error;
}

double TwoTurtle::orientation_error(){
	// Remapeamento do ângulo theta da turtle2 entre 0 e 360°
    double theta = (turtle2_pose->theta < 0) ? turtle2_pose->theta+2*M_PI : turtle2_pose->theta;


	// Remapeamento do ângulo theta da turtle1 entre 0 e 360°
    double yaw_target = (turtle1_pose->theta < 0) ? turtle1_pose->theta+2*M_PI : turtle1_pose->theta;

	// Obtenção do erro de angulo em módulo
    double error = abs(yaw_target - theta);

	// Adição do sinal de erro baseado na menor distância angular
	error = ((yaw_target > theta && error > M_PI) || (yaw_target < theta && error < M_PI)) ? -error : error;

    return error;
}

// Callback de recebimento da posição da turtle1
void TwoTurtle::pose_1_callback(const turtlesim::msg::Pose::SharedPtr target1){
	// Flag para acusar recebimento da posição da turtle1
	pose_1_sub_ok = true;
	turtle1_pose = target1;

	// Caso as flags acusem o recebimento das duas posições, é chamada a função de comando da turtle2
	if(pose_1_sub_ok && pose_2_sub_ok)
		this->follow_turtle();
}

// Callback de recebimento da posição da turtle2
void TwoTurtle::pose_2_callback(const turtlesim::msg::Pose::SharedPtr target2){
	// Flag para acusar recebimento da posição da turtle2
	pose_2_sub_ok = true;
	turtle2_pose = target2;

	// Caso as flags acusem o recebimento das duas posições, é chamada a função de comando da turtle2
	if(pose_1_sub_ok && pose_2_sub_ok)
		this->follow_turtle();
}

void TwoTurtle::follow_turtle(){
	// Reseta as flags
	pose_1_sub_ok = pose_2_sub_ok = false;

	// Declaração da mensagem do publisher e das velocidades lineares e angulares
	auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
	double linear_speed = 0;
	double angular_speed = 0;

	// Cálculo da velocidade linear com raio marginal de 0.1
	// As velocidades linear e angular são proporcionais ao erro de distância linear 
	// e de erro de ângulo, com ganhos Kp_L e Kp_A, respectivamente
	if(this->linear_error() > 0.1){
		linear_speed = this->linear_error()*Kp_L;
		angular_speed = this->yaw_error()*Kp_A;
	}

	// Cálculo da velocidade linear com erro angular de 0.01
	// A velocidade angular é proporcional ao erro de distância angular com ganho Kp_A
	else if(abs(this->orientation_error()) > 0.01){
		linear_speed = 0;
		angular_speed = this->orientation_error()*Kp_A;
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
	rclcpp::spin(std::make_shared<TwoTurtle>());

	// Encerramento nó
	rclcpp::shutdown();
}