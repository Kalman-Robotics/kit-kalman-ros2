# DOCUMENTACION DEL PAQUETE URDF DEL ROBOT (`kalman_description`)

Este paquete contiene la descripción física del robot en formato URDF (Unified Robot Description Format) utilizando XACRO, una herramienta que permite escribir URDFs de manera modular y parametrizada. El robot descrito se llama `uvrobot `.

* [Visualización en RVIZ](#visualizacion-en-rviz)
* [Descripciones Principales](#descripciones-principales)
* [Estructura del paquete](#estructura-del-paquete)
* [Configuración](#configuración)

---

## Visualización en RViz

### Comandos de Ejecución
Para lanzar la visualización del modelo URDF en RViz:
```bash
# Compilación (con symlinks para desarrollo)
colcon build --symlink-install

# Lanzamiento
ros2 launch kalman_description display_rviz.launch.py
```
**Salida esperada**:
- Ventana de RViz con el modelo del robot.
- Interfaz `joint_state_publisher_gui` para manipular articulaciones (si existen).

### Generación del URDF Final
Procesa el archivo XACRO para obtener el URDF estándar:
```bash
cd ~/uvrobot_ws/src/uvrobot-ros/kalman_description/urdf/
xacro uvrobot.urdf.xacro >> uvrobot.urdf  # Genera el URDF
```
**Uso**: Este archivo `uvrobot.urdf` puede ser cargado por ROS 2, RViz o Gazebo.

## Descripciones Principales

| Tipo |  | Función |
| --- | ---| --- |
| Dependencia| `urdf`, `xacro`, `rviz2`, `joint_state_publisher_gui` |
Nodo | <li>`robot_state_publisher`<li>`joint_state_publisher_gui` | <li>Publica TF y /robot_description<li> Interfaz para mover articulaciones manualmente |


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

## Configuración 

### Creación del paquete
Para crear el paquete en el espacio de trabajo, ejecuta:
```bash
cd ~/uvrobot_ws/src/uvrobot-ros 
ros2 pkg créate --build-type ament_cmake kalman_description --dependecies xacro std_msgs 
```
**Nota**: Asegúrate de declarar correctamente las dependencias en el CMakeLists.txt para evitar problemas al compilar en diferentes sistemas.

## Configuración de `CMakeLists.txt`

Dentro de este paquete se crea la carpeta `launch`:`~/urdf/xacro/`

Agrega los siguientes directorios al CMakeLists.txt para que se instalen correctamente:
```bash
install (DIRECTORY
    launch
    urdf
    #meshes  
    rviz
    DESTINATION share/${PROJECT_NAME}
)
```
**Nota:** *meshes*, se utilizan principalmente para definir la geometría visual y de colisión de los elementos del robot.

---

### Configuración de Archivos XACRO

#### Inclusión de Módulos y Librerías
El archivo principal **`uvrobot.urdf.xacro`** actúa como punto de integración para:
- **Componentes modulares**:  
  ```xml
  <xacro:include filename="$(find kalman_description)/urdf/xacro/chasis.xacro"/>
  <xacro:include filename="$(find kalman_description)/urdf/xacro/ruedas.xacro"/>
  ```
- **Plugins y extensiones**:  
  Incluye configuraciones para ROS 2 Control, sensores y simulaciones en Gazebo.

#### Propiedades y Macros
- **Parámetros reutilizables** definidos en cada submódulo (ej. en `wheels.xacro`):
  ```xml
  <xacro:property name="wheel_radius" value="0.1"/>  <!-- Radio en metros -->
  <xacro:property name="chassis_length" value="0.5"/>
  ```
- **Macros para construcción dinámica**:  
  Se invocan en el archivo principal `uvrobot.urdf.xacro` para ensamblar el robot:
  ```xml
  <!--construcción del robot-->
  <xacro:create_chassis/>
  <xacro:create_wheel prefix="left"/>
  ```


### Propósito del Archivo XACRO

### Ventajas Clave
| Característica          | Beneficio                                                                 |
|-------------------------|---------------------------------------------------------------------------|
| **Modularidad**         | <li>Permite ividir el modelo del robot<li>Divide el robot en componentes (chasis, ruedas) para mantenimiento fácil. |
| **Parametrización**     | Permite ajustar dimensiones o propiedades. Ej. `wheel_radius`, como la orientacion de las ruedas, sin modificar código.    |
| **simulación-Integración Gazebo**  | Listo para simulación física con plugins de sensores/actuadores.          |


---

### Definiciones Técnicas
#### Archivos XACRO (XML Macros)
Es una extensión de los archivos URDF (Unified Robot Description Format) utilizada en ROS para describir modelos de robots de forma más modular, reutilizable y parametrizada.

#### Estructura de un Archivo XACRO
1. **Encabezado**  
   * Define que es un archivo XML y usa el espacio de nombres de XACRO.
   ```xml
   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="uvrobot">
   ```

2. **Propiedades**  
   * Permiten definir constantes o variables para reutilizar valores (longitudes, masas, etc.). 

   Variables para dimensiones físicas:
   ```xml
   <xacro:property name="mass_wheel" value="2.0"/>  <!-- Masa en kg -->
   ```

3. **Macros**  
   * Bloques de código reutilizables que parametrizan definiciones de links o joints.
   * Esta línea de código se hace dentro de cada definición modular *.xacro*
   * Plantillas reutilizables (ej. para ruedas):
   ```xml
   <xacro:macro name="uvrobot">
       <link name="${prefix}_wheel">
           <!-- Geometría visual/colisión -->
       </link>
   </xacro:macro>
   ```

4. **Links y Joints**  
   - **Link**: 
      * Define partes físicas con inercia y geometría.
      * Representan componentes físicos del robot (partes rígidas como brazos, ruedas, etc.).
      * Contienen:
         * **Inercia** (para simulación física).
         * **Geometría visual** (para visualización en RViz).
         * **Geometría de colisión** (para simulación en Gazebo).
      Por ejemplo: 
     ```xml
     <link name="base_link">
         <visual>
             <geometry><box size="${length} ${width} ${height}"/></geometry>
         </visual>
     </link>
     ```
   - **Joint**: 
      * Conecta links con restricciones cinemáticas.
      * Definen las conexiones entre links, especificando el tipo de articulación (fija, giratoria, prismática, etc.).
      * Incluyen:
         * **Parent y child** (los links conectados).
         * **Origin** (posición y orientación relativa).
         * **Axis** (para articulaciones giratorias o prismáticas).
      Por ejemplo: 
     ```xml
     <joint name="wheel_joint" type="continuous">
         <parent link="base_link"/>
         <child link="wheel"/>
         <axis xyz="0 1 0"/>
     </joint>
     ```
