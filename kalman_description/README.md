# kalman_description package

Este paquete contiene la descripción física del robot en formato URDF (Unified Robot Description Format) utilizando XACRO, una herramienta que permite escribir URDFs de manera modular y parametrizada. El robot descrito se llama `uvrobot`.

- [kalman\_description package](#kalman_description-package)
  - [Uso](#uso)
    - [Visualización del modelo URDF en RViz](#visualización-del-modelo-urdf-en-rviz)
  - [Estructura del paquete](#estructura-del-paquete)
  - [Generación del URDF Final](#generación-del-urdf-final)


---

## Uso 

### Visualización del modelo URDF en RViz
```bash
ros2 launch kalman_description display_rviz.launch.py
```
**Salida esperada**:
- Ventana de RViz con el modelo del robot.
- Interfaz `joint_state_publisher_gui` para manipular articulaciones (si existen).


## Estructura del paquete
```bash
kalman_description/
├── launch/                         # Archivos de lanzamiento (launch files)
├── urdf/                           # Archivos XACRO/URDF del robot
│   ├── xacro/                      # Definiciones modulares del robot
│   │   ├── caster_wheel.xacro      
│   │   ├── chassis.xacro
│   │   ├── inertial.xacro
│   │   ├── propierties.xacro
│   │   └── wheel.xacro
│   └── uvrobot.urdf.xacro          # Archivo principal XACRO
├── rviz/                           # Configuraciones de RViz
├── meshes/                         # Mallas 3D (opcional)
└── CMakeLists.txt                  # Configuración del paquete
```


## Generación del URDF Final
Procesa el archivo XACRO para obtener el URDF estándar:
```bash
cd ~/.../kalman_description/urdf/
xacro uvrobot.urdf.xacro >> uvrobot.urdf  # Genera el URDF
```
**Uso**: Este archivo `uvrobot.urdf` puede ser cargado por ROS 2, RViz o Gazebo.