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

//Definição dos valores máximos de velocidade para o simulador turtlesim
#define LINEAR_SPEED_MAX 2.0
#define ANGULAR_SPEED_MAX 2.0
#define Kp_L 0.5
#define Kp_A 0.85

// Declaração do nó de teleoperação para o simulador turtlesim
class TwoTurtle : public rclcpp::Node{

	public:

		TwoTurtle() : Node("two_turtle"){	
			// Definição da qualidade de serviço para a comunicação do tópico		
			auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      /* CHAMAR SERVIÇO DE SPAW DA TURTLE2 */

      turtle_spawn = this->create_client<turtlesim::srv::Spawn>("spawn");
      auto new_turtle = std::make_shared<turtlesim::srv::Spawn::Request>();
      new_turtle->x = 1.0;
      new_turtle->y = 1.0;
      new_turtle->theta = 0.0;
      new_turtle->name = "turtle2";

      while (!turtle_spawn->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /spawn service");
      }
    
      auto result = turtle_spawn->async_send_request(new_turtle);

			// Instanciação do publisher do tópico cmd_vel, com mensagem do tipo Twist
			cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", default_qos);

      pose_1_sub = this->create_subscription<turtlesim::msg::Pose>(
                                "/turtle1/pose", default_qos, 
                                std::bind(&TwoTurtle::pose_1_callback, this, _1));

      pose_2_sub = this->create_subscription<turtlesim::msg::Pose>(
                                "/turtle2/pose", default_qos, 
                                std::bind(&TwoTurtle::pose_2_callback, this, _1));

			RCLCPP_INFO(this->get_logger(), "ECAi21 | Two Turtle");
		}

		~TwoTurtle(){
			RCLCPP_INFO(this->get_logger(), "Shutting down Node");
		}

	private:

		void pose_1_callback(const turtlesim::msg::Pose::SharedPtr target1);
		void pose_2_callback(const turtlesim::msg::Pose::SharedPtr target2);
		void follow_turtle();
    void show_pose();
    double linear_error();
    double yaw_error();
    double orientation_error();
    double modulo(const turtlesim::msg::Pose::SharedPtr target);
    double produto_interno(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2);

		// Declaração do publisher
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_1_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_2_sub;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr turtle_spawn;

    turtlesim::msg::Pose::SharedPtr turtle1_pose;
    turtlesim::msg::Pose::SharedPtr turtle2_pose;

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

double TwoTurtle::linear_error(){
    return sqrt((turtle1_pose->x - turtle2_pose->x)*(turtle1_pose->x - turtle2_pose->x) + (turtle1_pose->y - turtle2_pose->y)*(turtle1_pose->y - turtle2_pose->y));
}

double TwoTurtle::modulo(const turtlesim::msg::Pose::SharedPtr target){
    return sqrt((target->x)*(target->x) + (target->y )*(target->y));
}

double TwoTurtle::produto_interno(const turtlesim::msg::Pose::SharedPtr target1,const turtlesim::msg::Pose::SharedPtr target2){
    return (target1->x)*(target2->x) + (target1->y)*(target2->y);
}

double TwoTurtle::yaw_error(){
    auto turtle1_pose_0 = std::make_shared<turtlesim::msg::Pose>();
    auto x_ref = std::make_shared<turtlesim::msg::Pose>();

    x_ref->x = 1;
    x_ref->y = 0;

    turtle1_pose_0->x = turtle1_pose->x - turtle2_pose->x;
    turtle1_pose_0->y = turtle1_pose->y - turtle2_pose->y;

    double yaw_target = (modulo(turtle1_pose_0)==0) ? 0 : copysign(acos(produto_interno(x_ref,turtle1_pose_0)/modulo(turtle1_pose_0)),turtle1_pose_0->y);

    return (yaw_target - turtle2_pose->theta);
}

double TwoTurtle::orientation_error(){
    return turtle1_pose->theta - turtle2_pose->theta;
}

void TwoTurtle::pose_1_callback(const turtlesim::msg::Pose::SharedPtr target1){
  pose_1_sub_ok = true;
  turtle1_pose = target1;

  if(pose_1_sub_ok && pose_2_sub_ok)
    this->follow_turtle();
}

void TwoTurtle::pose_2_callback(const turtlesim::msg::Pose::SharedPtr target2){
  pose_2_sub_ok = true;
  turtle2_pose = target2;

  if(pose_1_sub_ok && pose_2_sub_ok)
    this->follow_turtle();
}

void TwoTurtle::follow_turtle(){
  pose_1_sub_ok = pose_2_sub_ok = false;

  auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
  double linear_speed = 0;
  double angular_speed = 0;

  // control law

  if(this->linear_error() > 0.1){

    linear_speed = this->linear_error()*Kp_L;

    angular_speed = this->yaw_error()*Kp_A;
  }
  else if(abs(this->orientation_error()) > 0.01){
    linear_speed = 0;
    angular_speed = this->orientation_error()*Kp_A;
  }

  cmd_msg->linear.x = (abs(linear_speed) > LINEAR_SPEED_MAX) ? copysign(LINEAR_SPEED_MAX,linear_speed) : linear_speed;
  cmd_msg->angular.z = (abs(angular_speed) > ANGULAR_SPEED_MAX) ? copysign(ANGULAR_SPEED_MAX,angular_speed) : angular_speed;


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
