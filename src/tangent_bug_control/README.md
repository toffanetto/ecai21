# Exercício 5 - Tangent Bug Control 

Descrição

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
