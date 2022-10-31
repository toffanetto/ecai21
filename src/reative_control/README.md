# Exercício 4 - Reative Control

Nó de desvio de obstáculos via controle reativo, utilizando um robô diferencial em ambiente simulado no software Gazebo, utilizando um LiDAR de 180° para percepção.

---

## Executando

Executar as linhas de código abaixo no terminal do diretório da workspace:

```
source /usr/share/gazebo/setup.sh

source install/source.bash

ros2 launch reative_control reative_control.py
```

## Dependências

- Gazebo
- RViz2
- [Dolly (Chapulina)](https://github.com/chapulina/dolly)
