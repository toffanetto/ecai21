# Exercício 7 - Feedback Linearization Control 

Implementação em C++ da estratégia de controle Feedback Linearization, utilizando odometria.

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
colcon build --packages-select feedback_linearization_control

source /usr/share/gazebo/setup.sh

source install/source.bash
```

Mapa com obstáculos:

```
ros2 launch feedback_linearization_control feedback_linearization_control.py world:=world_4.world
```

Mapa com armadilha de mínimo local:

```
ros2 launch feedback_linearization_control feedback_linearization_control.py world:=world_5.world
```

## Dependências

- Gazebo
- RViz2
- [Dolly (Chapulina)](https://github.com/chapulina/dolly)
