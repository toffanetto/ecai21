# Exercício 4 - Reactive Control

Nó de desvio de obstáculos via controle reativo, utilizando um robô diferencial em ambiente simulado no software Gazebo, embarcado de um LiDAR de 180° para percepção do ambiente.

---

## Executando

Executar as linhas de código abaixo no terminal do diretório da workspace:

```
colcon build --packages-select reactive_control

source /usr/share/gazebo/setup.sh

source install/source.bash

ros2 launch reactive_control reactive_control.py
```




## Dependências

- Gazebo
- RViz2
- [Dolly (Chapulina)](https://github.com/chapulina/dolly)
