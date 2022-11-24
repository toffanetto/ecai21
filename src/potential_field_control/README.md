# Exercício 6 - Potential Field Control 

Descrição

---

## Executando

Executar as linhas de código abaixo no terminal do diretório da workspace:

```
colcon build --packages-select potential_field_control

source /usr/share/gazebo/setup.sh

source install/source.bash

ros2 launch potential_field_control potential_field_control.py
```

## Dependências

- Gazebo
- RViz2
- [Dolly (Chapulina)](https://github.com/chapulina/dolly)
