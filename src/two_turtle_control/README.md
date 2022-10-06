# Exercício 2 - Pega Pega Turtlesim

Nó de operação autônoma de uma turtle para seguir a outra, sendo a tartaruga de referência teleoperada pelo pacote `teleop_cmd_vel`

---

`ros2 run two_turtle_control two_turtle`

## Launch File

O launch file inicializa tanto o de pega pega, quanto o nó teleop de teleoperação e o turtlesim_node, abrindo uma janela para o simulador e outra para a teleoperação, além de mostrar a posição das turtles no terminal principal.

`ros2 launch two_turtle_control two_turtle.py`

> O arquivo launch file utiliza o Konsole como janela para o teleop, podendo não funcionar sem esse terminal
