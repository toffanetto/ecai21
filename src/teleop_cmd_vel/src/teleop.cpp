#include <cstdio>
#include <memory>
#include <chrono>
#include <assert.h>

#include "kbhit.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

//Definição dos valores máximos de velocidade para o simulador turtlesim
#define LINEAR_SPEED_MAX 2
#define ANGULAR_SPEED_MAX 2

using namespace std::chrono_literals;

// Declaração do nó de teleoperação para o simulador turtlesim
class Teleop : public rclcpp::Node{

	public:

		Teleop() : Node("teleop"){	
			// Definição da qualidade de serviço para a comunicação do tópico		
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

			// Instanciação do publisher do tópico cmd_vel, com mensagem do tipo Twist
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", default_qos);

			// Instanciação do timer para criação de base de tempo para leitura do teclado e publicação das mensagens
			// chamando a função readKey_callback()
			timer_key_read = this->create_wall_timer(100ms, std::bind(&Teleop::readKey_callback, this));

			RCLCPP_INFO(this->get_logger(), "ECAi21 | Turtle Teleop");
		}

		~Teleop(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}

	private:

		// Função de callback do timer timer_key_read
		void readKey_callback(){

			// Declaração da mensagem que será enviada
			auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();

			// Leitura da caracter pressionada por meio da função kbhit() e tratamento da saída
			if (kbhit()){

				// Armazena a caractere digitada
				char c = tolower(getchar());

				RCLCPP_INFO(this->get_logger(), "Pressed key %c",c);

				// Realiza o tratamento da caractere digitada por meio dos parametros de velocidade linear e angular
				// salvos em linear_speed e angular_speed, que representam a velocidade do robô, e pelos parametros 
				// unitários de direção do movimento, armazenado em linear_speed_dir e angular_speed_dir. 
				// Para valor positivo de linear_speed_dir, o robò se movimenta para frente, e negativo, para trás.
				// Para valor positivo de angular_speed_dir, o robò se movimenta no sentido anti-horário, e negativo
				// no sentido horário.
				// Os valores de linear_speed e angular_speed são limitados entre 0 e os valores máximos para cada
				// variável, dados por LINEAR_SPEED_MAX e ANGULAR_SPEED_MAX.
				switch (c){

					// Para frente
					case 'w':
						linear_speed_dir = 1;
						angular_speed_dir = 0;
						break;

					// Para trás
					case 'x':
						linear_speed_dir = -1;
						angular_speed_dir = 0;
						break;

					// Rotação anti-horária
					case 'a':
						linear_speed_dir = 0;
						angular_speed_dir = 1;
						break;

					// Rotação horária
					case 'd':
						linear_speed_dir = 0;
						angular_speed_dir = -1;
						break;

					// Para frente com rotação anti-horária
					case 'q':
						linear_speed_dir = 1;
						angular_speed_dir = 1;
						break;

					// Para trás com rotação anti-horária
					case 'z':
						linear_speed_dir = -1;
						angular_speed_dir = 1;
						break;

					// Para frente com rotação horária
					case 'e':
						linear_speed_dir = 1;
						angular_speed_dir = -1;
						break;

					// Para trás com rotação horária
					case 'c':
						linear_speed_dir = -1;
						angular_speed_dir = -1;
						break;

					// Parado
					case 's':
						linear_speed_dir = 0;
						angular_speed_dir =0;
						break;

					// Incremento de velocidade linear
					case '1':
						linear_speed = (linear_speed+0.2 >= LINEAR_SPEED_MAX) ? LINEAR_SPEED_MAX : linear_speed+0.2;
						break;

					// Decremento de velocidade linear
					case '2':
						linear_speed = (linear_speed-0.2 <= 0) ? 0 : linear_speed-0.2;
						break;

					// Incremento de velocidade angular
					case '3':
						angular_speed = (angular_speed+0.2 >= ANGULAR_SPEED_MAX) ? ANGULAR_SPEED_MAX : angular_speed+0.2;
						break;

					// Decremento de velocidade angular
					case '4':
						angular_speed = (angular_speed-0.2 <= 0) ? 0 : angular_speed-0.2;
						break;
					
					// Para o robô e mata o nó
					case 'p':			
						cmd_msg->linear.x = 0;
						cmd_msg->angular.z = 0;
						cmd_pub->publish(std::move(cmd_msg));
						rclcpp::shutdown();
						break;				
					default:
						break;
				}
        	}

			// Atualiza o valor de velocidade linear em x no corpo da mensagem a ser publicada
			cmd_msg->linear.x = linear_speed*linear_speed_dir;

			// Atualiza o valor de velocidade angular em z no corpo da mensagem a ser publicada
			cmd_msg->angular.z = angular_speed*angular_speed_dir;

			// Publica a mensagem
			cmd_pub->publish(std::move(cmd_msg));
		
		}

		// Declaração do publisher
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

		// Declaração do timer
		rclcpp::TimerBase::SharedPtr timer_key_read;

		// Declaração de variáveis de velocidade
		float linear_speed = 0.0;
		float angular_speed = 0.0;
		int linear_speed_dir = 0;
		int angular_speed_dir = 0;
    
};

int main(int argc, char ** argv){
	
	// Inicialização do ROS2
	rclcpp::init(argc, argv);

	// Inicialização do nó em um executor de forma ciclica
	rclcpp::spin(std::make_shared<Teleop>());

	// Encerramento nó
	rclcpp::shutdown();
}
