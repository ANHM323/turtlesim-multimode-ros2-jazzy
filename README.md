# Turtlesim Multi-Mode Controller (ROS2 Jazzy)

Implementación en Python de un nodo de control para turtlesim con tres modos de operación, desarrollado para la práctica final de ROS2 Part 2.

## Decisiones de Diseño (Respuestas a la Guía 7.3)

1. Selección de modo: Se utiliza un **Servicio** (`/set_mode`). Es una operación síncrona discreta que requiere confirmación inmediata de éxito/fallo.
2. Modo Manual: El nodo **deja de publicar** en `/turtle1/cmd_vel`. Esto evita conflictos de sobredeterminación cinemática, permitiendo que herramientas externas (teleop, rqt) tengan control exclusivo.
3. Modo Círculos: Se implementa con un **Tópico + Timer**. Publica velocidades lineales y angulares constantes (lazo abierto). Al salir del modo, se publica un Twist nulo para detener la tortuga de forma segura.
4. Modo Trayectoria: Se utiliza una **Acción** (`/navigate_waypoints`). Es la interfaz adecuada para procesos de larga duración que requieren feedback en tiempo real, capacidad de cancelación (preemption) y notificación de finalización.
5. Consulta de modo actual: Se implementó el servicio `/get_mode` para consultar el estado interno de la máquina de estados en cualquier momento.

## Estructura del Repositorio

```text
ros2_ws/
└── src/
    ├── turtlesim_interfaces/       # Paquete ament_cmake (interfaces)
    │   ├── action/
    │   │   └── NavigateWaypoints.action
    │   ├── srv/
    │   │   ├── GetMode.srv
    │   │   └── SetMode.srv
    │   ├── CMakeLists.txt
    │   ── package.xml
    └── turtlesim_control/          # Paquete ament_python (lógica)
        ├── resource/
        │   └── turtlesim_control
        ├── turtlesim_control/
        │   ├── __init__.py
        │   ── mode_controller.py
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## Instalación y Ejecución

1. Compilar el workspace:
   cd ~/ros2_ws
   colcon build
   source install/setup.bash

2. Abrir 3 terminales y ejecutar en cada una:
   source ~/ros2_ws/install/setup.bash

3. Terminal 1 (Simulador):
   ros2 run turtlesim turtlesim_node

4. Terminal 2 (Nodo de control):
   ros2 run turtlesim_control mode_controller

5. Terminal 3 (Pruebas):
   # Consultar modo
   ros2 service call /get_mode turtlesim_interfaces/srv/GetMode "{}"
   
   # Cambiar a círculos (horario o antihorario)
   ros2 service call /set_mode turtlesim_interfaces/srv/SetMode "{mode: 'circle_cw'}"
   
   # Cambiar a trayectoria y enviar goal (3 puntos)
   ros2 service call /set_mode turtlesim_interfaces/srv/SetMode "{mode: 'trajectory'}"
   ros2 action send_goal /navigate_waypoints turtlesim_interfaces/action/NavigateWaypoints "{x: [5.0, 8.0, 2.0], y: [5.0, 2.0, 8.0]}" --feedback

## Entregables

- Informe Técnico (Formato IEEE): [INSERTAR ENLACE AQUÍ]
- Video de Demostración: [INSERTAR ENLACE AQUÍ]
- Evidencias gráficas (RQT Graph): Ver carpeta docs/media/
