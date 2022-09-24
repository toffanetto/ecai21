# Exercício 1 - Movimentação em n-direções pelo teclado em C++

Nó de teleoperação para o simulador turtlesim baseado em comando enviado via teclado.

---

`ros2 run teleop_cmd_vel teleop`

## Launch File

O launch file inicializa tanto o nó teleop de teleoperação quanto o turtlesim_node no mesmo contexto (namespace), abrindo uma janela para o simulador e outra para a teleoperação.

`ros2 launch teleop_cmd_vel teleop_turtle.py`

> O arquivo launch file utiliza o Konsole como janela para o teleop, podendo não funcionar sem esse terminal
