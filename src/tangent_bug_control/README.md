# Exercício 5 - Tangent Bug Control 

Implementação em C++ da estratégia de navegação de Tangent Bug, utilizando odometria e LiDAR de 180°.

## Diretórios e Arquivos

- `/docs`: Documentação da implementação da estratégia;
- `/env-hooks`: Arquivos de configuração da integração entre o Gazebo e o projeto via launch file;
- `/launch`: Launch file para inicialização do ambiente simulado juntamente do ROS 2;
- `/models`: Descrição em SDF dos obstáculos, modelos e robôs utilizados;
- `/rviz`: Confiuração do ambiente RViz 2 para visualização da instrumentação do robô;
- `/scr`: Código fonte;
- `/worlds`: Mundos utilizados no simulador Gazebo;

---

## Executando

Executar as linhas de código abaixo no terminal do diretório da workspace:

```
colcon build --packages-select tangent_bug_control

source /usr/share/gazebo/setup.sh

source install/source.bash

ros2 launch tangent_bug_control tangent_bug_control_control.py
```

## Dependências

- Gazebo
- RViz2
- [Dolly (Chapulina)](https://github.com/chapulina/dolly)
