# Exercício 3 - Turtle Avoid

Nó de desvio de obstáculos, onde uma tartaruga é centrada no espaço, e a outra tartaruga deve seguir à pontos alvos sorteados aleatóriamente pelo mapa sem colidir com a tartaruga nem com as paredes.

---

`ros2 run turtle_avoid_control turtle_avoid`

## Launch File

O launch file inicializa tanto o de desvio, quanto o turtlesim_node, abrindo uma janela para o simulador, além de mostrar a posição das turtles no terminal principal.

`ros2 launch turtle_avoid_control turtle_avoid.py`

> O arquivo launch file utiliza o Konsole como janela para o teleop, podendo não funcionar sem esse terminal
