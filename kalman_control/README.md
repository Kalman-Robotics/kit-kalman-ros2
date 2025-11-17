# DOCUMENTACION DEL PAQUETE ROS2_CONTROL DEL ROBOT (`uvrobot_control`)

Este paquete ROS 2 permite simular el robot **UV-ROBOT** en el entorno **Gazebo**. Incluye mundos, modelos, y scripts de lanzamiento para automatizar la inserción del robot y la publicación de su estado.

---
* [Descripciones Principales](#descripciones-principales)

---

## Descripciones Principales

| Tipo |  | Función |
| --- | ---| --- |
| Dependencia| `controller_manager, diff_drive_controller, joint_state_broadcaster, tf2` |
Nodo | <li>`ros2_control_node`<li>`controller_manager` | <li>Nodo central de ros2_control<li>Gestiona controladores (diferencial, joint states) |