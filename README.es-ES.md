

# Sistema de Navegación en Entornos sin GPS

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-v1.14-green.svg)](https://px4.io/)
[![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)](https://www.python.org/)

**Navegación autónoma de drones utilizando flujo óptico, IMU y lidar cuando no está disponible el GPS.**

---

## 📋 Descripción General

Este proyecto permite que los drones naveguen de forma autónoma en entornos sin GPS con una precisión centimétrica mediante fusión multisensor y algoritmos de SLAM. El sistema está diseñado para misiones de inspección de almacenes, búsqueda y rescate, y mapeo.

---

## 🎯 Lista de Características

- [x] Adquisición de datos multisensor (Flujo óptico, IMU, Lidar)
- [x] SLAM en tiempo real con mapeo de cuadrícula de ocupación
- [x] Planificación de trayectoria local y global con evasión de obstáculos
- [x] Integración offboard con PX4 mediante MAVSDK
- [x] Mantenimiento de posición y navegación por waypoints basados en visión
- [x] Mapeo multisesión (Persistencia de mapas)
- [x] Filtrado de objetos dinámicos
- [x] Estimación de poses por visión para EKF2
- [x] Aterrizaje automático de seguridad (fail-safe)
- [x] Seguimiento de trayectoria con Pure Pursuit

---

## 🏗️ Desglose de Características

### Épica 1: Adquisición y Fusión de Datos Multisensor
| ID Característica | Nombre de la Característica | Descripción | Prioridad |
|------------|--------------|-------------|----------|
| 1.1.1 | Integración de Flujo Óptico | Estimación de velocidad en tiempo real desde cámara orientada hacia abajo | P0 |
| 1.1.2 | Procesamiento de datos IMU | Estimación de pose de alta frecuencia y filtrado de vibraciones | P0 |
| 1.1.3 | SLAM de Lidar 2D | Mapeo y localización en plano horizontal | P0 |
| 1.1.4 | Sincronización de sensores | Alineación de marcas de tiempo a nivel de microsegundos entre sensores | P1 |

### Épica 2: Motor de SLAM en Tiempo Real
| ID Característica | Nombre de la Característica | Descripción | Prioridad |
|------------|--------------|-------------|----------|
| 1.1.6 | SLAM basado en grafos | Cierre de bucle y optimización de grafo de poses | P0 |
| 1.1.7 | Mapeo de cuadrícula de ocupación | Representación del entorno en 2D/2.5D | P0 |
| 1.1.8 | Filtrado de objetos dinámicos | Eliminación de obstáculos móviles del mapa estático | P1 |
| 1.1.9 | Persistencia de mapas | Guardar/cargar mapas para misiones repetidas | P1 |

### Épica 3: Pila de Navegación Autónoma
| ID Característica | Nombre de la Característica | Descripción | Prioridad |
|------------|--------------|-------------|----------|
| 1.1.11 | Planificador de trayectoria local | Enfoque de ventana dinámica para evasión en tiempo real | P0 |
| 1.1.12 | Planificador de trayectoria global | Algoritmo A* sobre cuadrícula de ocupación | P0 |
| 1.1.13 | Modo de mantenimiento de posición | Vuelo estable sin deriva GPS | P0 |
| 1.1.14 | Navegación por waypoints | Ejecución de waypoints predefinidos | P1 |

### Épica 4: Integración con Controlador de Vuelo
| ID Característica | Nombre de la Característica | Descripción | Prioridad |
|------------|--------------|-------------|----------|
| 1.1.16 | Modo Offboard MAVSDK | Envío de puntos de consola de posición vía MAVLink | P0 |
| 1.1.17 | Estimación de poses por visión | Inyección de pose de SLAM en EKF2 | P0 |
| 1.1.18 | Lógica de seguridad (Fail-safe) | Aterrizaje automático ante fallo de SLAM | P0 |

---
## 🔧 Arquitectura Técnica

![GPS-Denied Navigation System](GPS-Denied%20Indoor%20Navigation%20System.drawio.png)

---

## 🔗 Integración del Sistema

### Flujo de Datos Completo

```
1. SENSOR ACQUISITION (Epic 1)
   ├─ optical_flow_integration → Visual velocity
   ├─ imu_processing → Orientation & acceleration
   └─ lidar_slam_2d → Laser scans
          ↓
2. SENSOR FUSION (Epic 1)
   └─ sensor_synchronization → Time-aligned data
          ↓
3. LOCALIZATION & MAPPING (Epic 2)
   ├─ graph_slam → Pose estimation with loop closure
   ├─ occupancy_grid_mapping → Environment representation
   ├─ dynamic_filter → Remove moving objects
   └─ map_persistence → Save/load maps
          ↓
4. PATH PLANNING (Epic 3)
   ├─ global_planner (A*) → Optimal waypoint paths
   └─ local_planner (DWA) → Real-time obstacle avoidance
          ↓
5. TRAJECTORY EXECUTION (Epic 3)
   ├─ trajectory_tracker (Pure Pursuit) → Smooth path following
   ├─ position_hold (PID) → Stable hovering
   └─ waypoint_navigation → Mission execution
          ↓
6. FLIGHT CONTROL (Epic 4)
   ├─ vision_pose_estimator → SLAM → EKF2 fusion
   ├─ mavsdk_offboard → Position setpoints via MAVLink
   └─ failsafe_controller → Emergency landing on failure
          ↓
7. VEHICLE EXECUTION
   └─ PX4 Flight Controller → Motor commands
```

### Matriz de Integración de Características

| Característica | Entradas | Salidas | Se Integra Con |
|---------|--------|---------|-----------------|
| **optical_flow_integration** | Fotogramas de cámara | `/optical_flow/velocity` | sensor_synchronization |
| **imu_processing** | Datos brutos IMU | `/imu/data` | sensor_synchronization, graph_slam |
| **lidar_slam_2d** | Escaneos láser | `/scan` | sensor_synchronization, graph_slam |
| **sensor_synchronization** | Todos los temas de sensores | `/fused/odom` | graph_slam |
| **graph_slam** | Odometría fusionada | `/slam/pose`, `/odom` | occupancy_grid_mapping, vision_pose_estimator |
| **occupancy_grid_mapping** | Pose SLAM + escaneos | `/map` | global_planner, local_planner |
| **dynamic_filter** | `/map` + velocidades | `/static_map` | map_persistence |
| **map_persistence** | Mapa filtrado | Mapas guardados | occupancy_grid_mapping |
| **global_planner** | `/static_map`, meta | `/global_plan` | trajectory_tracker |
| **local_planner** | `/map`, `/odom` | `/local_plan` | trajectory_tracker |
| **trajectory_tracker** | Planes + odometría | `/cmd_vel` | mavsdk_offboard |
| **position_hold** | Pose objetivo | `/cmd_vel_hold` | mavsdk_offboard |
| **waypoint_navigation** | Archivo de misión | `/goal_pose` | global_planner |
| **vision_pose_estimator** | `/odom` | `/mavros/vision_pose/pose` | EKF2 de PX4 |
| **mavsdk_offboard** | Temas `/cmd_vel` | `/mavros/setpoint_raw/local` | PX4 |
| **failsafe_controller** | `/odom`, `/mavros/state` | `/failsafe/active`, modo LAND | PX4 |

### Dependencias Críticas

**Para Localización**:
- optical_flow_integration + imu_processing + lidar_slam_2d → sensor_synchronization → graph_slam

**Para Navegación**:
- graph_slam → occupancy_grid_mapping → global_planner + local_planner → trajectory_tracker

**Para Control de Vuelo**:
- trajectory_tracker OR position_hold → mavsdk_offboard → PX4
- graph_slam → vision_pose_estimator → PX4 EKF2

**Para Seguridad**:
- graph_slam → failsafe_controller → PX4 (aterrizaje de emergencia)

---

## ⚠️ Mitigación de Riesgos

| Riesgo | Probabilidad | Impacto | Mitigación |
|------|-------------|--------|------------|
| Deriva de SLAM en áreas sin características | Alta | Crítico | Agregar hitos AprilTag, fusionar múltiples modalidades de sensores |
| Sobrecarga computacional | Media | Alto | Descargar trabajo a Coral TPU, optimizar con nodos en C++ |
| Interferencia magnética | Alta | Medio | Usar flujo óptico como fuente principal de guiñada (yaw), desactivar fusión magnética |
| Distorsión por movimiento del lidar | Media | Medio | Implementar compensación de movimiento usando pre-integración de IMU |

---

## ✅ Criterio de Finalización

- ✓ El drone mantiene un vuelo estable durante 2 minutos en una habitación de 5m×5m (<20 cm de deriva)
- ✓ Navega con éxito una misión de 4 waypoints con evasión de obstáculos
- ✓ El SLAM se relocaliza después de ser levantado y movido (problema del robot secuestrado)
- ✓ Todas las pruebas de seguridad superan los resultados en simulación + validación en mundo real
- ✓ La documentación incluye procedimientos de calibración y guía de ajuste

---

## 🚀 Inicio Rápido

### Prerrequisitos
- Controlador de vuelo compatible con PX4 (Pixhawk 4, Cube Orange)
- Raspberry Pi 4 (4GB+)
- RPLidar A1/A2, sensor de flujo óptico (PMW3901)
- ROS2 Humble, MAVSDK-Python

### Instalación
```bash
# Navigate to project directory
cd "Autonomous Navigation & Control"

# Build all packages (16 ROS2 packages)
./setup_workspace.sh

# Source workspace
source install/setup.bash
```

### Primer Vuelo - Secuencia Completa de Inicio

```bash
# 1. Launch sensor fusion (Terminal 1)
ros2 launch sensor_synchronization sync.launch.py

# 2. Launch SLAM system (Terminal 2)
ros2 launch graph_slam graph_slam.launch.py

# 3. Launch occupancy grid mapping (Terminal 3)
ros2 launch occupancy_grid_mapping mapping.launch.py

# 4. Launch MAVROS - connect to PX4 (Terminal 4)
ros2 launch mavros px4.launch fcu_url:=udp://:14540@

# 5. Launch vision pose estimator (Terminal 5)
ros2 launch vision_pose_estimator vision_pose.launch.py

# 6. Launch failsafe controller (Terminal 6)
ros2 launch failsafe_controller failsafe.launch.py

# 7. Launch navigation stack (Terminal 7)
ros2 launch global_planner planner.launch.py &
ros2 launch local_planner planner.launch.py &
ros2 launch trajectory_tracker tracker.launch.py

# 8. Launch offboard control (Terminal 8)
ros2 launch mavsdk_offboard offboard.launch.py

# 9. Execute waypoint mission (Terminal 9)
ros2 launch waypoint_navigation waypoint.launch.py \
  mission_file:=missions/example_mission.yaml \
  auto_start:=true
```

### Inicio Rápido - Prueba de Mantenimiento de Posición

```bash
# Minimal setup for testing position hold
./setup_workspace.sh
source install/setup.bash

# Launch SLAM + Position Hold
ros2 launch graph_slam graph_slam.launch.py &
ros2 launch mavros px4.launch fcu_url:=udp://:14540@ &
ros2 launch vision_pose_estimator vision_pose.launch.py &
ros2 launch position_hold hold.launch.py &
ros2 launch failsafe_controller failsafe.launch.py
```

---

## 📦 Paquetes Implementados (16 en Total)

### Épica 1: Adquisición y Fusión de Datos Multisensor ✅

1. **optical_flow_integration** - Odometría visual desde cámara
   - Estimación de velocidad a 20 Hz
   - Soporte de covarianza
   - Modo simulación

2. **imu_processing** - Datos de orientación de alta tasa
   - Procesamiento IMU a 100 Hz
   - Filtro complementario
   - Soporte de calibración

3. **lidar_slam_2d** - SLAM láser 2D
   - Correspondencia de escaneos en tiempo real
   - Difusión de TF
   - Detección de cierre de bucle

4. **sensor_synchronization** - Fusión multisensor
   - Ventana de tiempo de 50 ms
   - Fusión con filtro de Kalman
   - Odometría sincronizada

### Épica 2: Motor de SLAM en Tiempo Real ✅

5. **graph_slam** - SLAM de optimización de grafos
   - Optimización de grafo de poses
   - Detección de cierre de bucle
   - Integración con G2O

6. **occupancy_grid_mapping** - Mapeo probabilístico
   - Ocupación en log-odds
   - Actualizaciones dinámicas
   - Resolución configurable

7. **dynamic_filter** - Filtrado de movimiento
   - Detección basada en velocidad
   - Generación de mapa estático
   - Ajuste de umbrales

8. **map_persistence** - Guardar/cargar mapas
   - Formato YAML/PGM
   - Soporte multisesión
   - Guardado automático

### Épica 3: Pila de Navegación Autónoma ✅

9. **local_planner** - Enfoque de Ventana Dinámica (DWA)
   - Evasión de obstáculos en tiempo real
   - Muestreo del espacio de velocidades
   - Verificación de colisiones

10. **global_planner** - Planificación de trayectoria A*
    - Búsqueda de trayectoria óptima
    - Búsqueda en cuadrícula de ocupación
    - Generación de waypoints

11. **position_hold** - Vuelo con PID
    - Control de posición 3D (XY + Z)
    - Tasa de control de 50 Hz
    - Compensación de deriva

12. **trajectory_tracker** - Pure Pursuit
    - Seguimiento suave de trayectoria
    - Visión previa (lookahead) adaptativa
    - Dirección basada en curvatura

13. **waypoint_navigation** - Ejecución de misión
    - Archivos de misión en YAML
    - Ejecución secuencial
    - Marcadores visuales

### Épica 4: Integración con Controlador de Vuelo ✅

14. **mavsdk_offboard** - Control MAVLink
    - Transmisión de puntos de consola de posición (20 Hz)
    - Gestión del modo offboard
    - Capacidad de armado automático

15. **vision_pose_estimator** - Integración con EKF2
    - Inyección de pose de SLAM
    - Publicación a 30 Hz
    - Soporte de covarianza

16. **failsafe_controller** - Monitor de seguridad
    - Verificación de estado de SLAM
    - Aterrizaje de emergencia automático
    - Advertencias multietapa

---

## 🤝 Contribuciones

1. Realice un fork del repositorio
2. Cree una rama de características (`git checkout -b feature/amazing-feature`)
3. Confirme sus cambios (`git commit -m 'Add amazing feature'`)
4. Publique en la rama (`git push origin feature/amazing-feature`)
5. Abra un Pull Request

---

## 📜 Licencia

Este proyecto está licenciado bajo la Licencia MIT - consulte el archivo [LICENSE](LICENSE) para obtener más detalles.

---

## 📚 Referencias y Recursos

- [Estimación de Poses por Visión en PX4](https://docs.px4.io/main/en/computer_vision/)
- [Caja de Herramientas SLAM para ROS2](https://github.com/SteveMacenski/slam_toolbox)
- [MAVSDK-Python](https://mavsdk.mavlink.io/main/en/python/)
- [Hector SLAM](http://wiki.ros.org/hector_slam)

---

**⚠️ Aviso de Seguridad:** Realice siempre pruebas en simulación primero. Utilice un piloto de seguridad con interruptor de anulación manual. Se requiere aprobación para vuelos en interiores.
