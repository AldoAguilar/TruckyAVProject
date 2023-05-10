# Trucky AV Project Configuración en ROS

Este directorio incluye los recursos necesarios para el control del robot "Trucky" con ROS, permitiendo el control de actuadores y lectura de sensores así como el intercambio de mensajes.

## Tabla de Contenidos

* [Instalación](#instalación)
  - [Prerrequisitos de hardware](#prerrequisitos-de-hardware)
  - [Prerrequisitos de software](#prerrequisitos-de-software)
  - [Configuración de software](#configuración-de-software)
* [Descripción de contenidos](#descripción-de-contenidos)
  - [packages/trucky_custom_msgs]
  - [workspaces/catkin_ws]

## Instalación
Estas indicaciones tienen como objetivo ayudar al usuario a replicar la configuración del proyecto en su estado acutal, así como proveer una descripción general del sistema implementado.

| NOTA: Esta documentación sirve como una guía de configuración del proyecto para su funcionamiento con el hardware y software específicado, sin embargo se recomienda que todo nuevo desarrollador realice el esfuerzo de actualizar esta documentación con los nuevos requisitos de hardware y de software para así garantizar el soporte del proyecto a largo plazo. |
| --- |

### Prerrequisitos de hardware
Para poder realizar la configuración y uso de los recursos del proyecto en ROS, se deberá contar con el robot debidamente armado y en funcionamiento. (Revisar los [requisitos de hardware del sistema](..) para mayor referencia) Esto incluye:
* Sistema eléctrico en correcto funcionamiento (baterias, reguladores, etc).
* Sistema de encoders en correcto funcionamiento.
* Sistema de perifericos en correcto funcionamiento. (Cámara(s), Lidar, etc).
* Tarjeta(s) de desarrollo (Jetson / Odroid) debidamente configruadas con Ubuntu 18.04.
* Computadora personal con Ubuntu 18.04 y con conexión de red directa con la(s) tarjeta(s) de desarrollo del robot.
* Driver de motores "Trucky Driver Board" montada y funcional.

### Prerrequisitos de software 
Los prerrequisitos de software (Revisar los [requisitos de software del sistema](..) para mayor referencia) para el uso de los recursos del proyecto en ROS incluyen:
* ROS Melodic debidamente instalado en la(s) tarjeta(s) de desarrollo del robot (Jetson / Odroid).
* ROS Melodic debidamente instalado en la computadora personal.

### Configuración de software 
#### Tarjeta(s) de desarrollo
##### Instalación de paquetes de ROS
El robot basa la integración del Driver de motores "Trucky Driver Board" con ROS mediante la comunicación serial con las tarjetas Arduino Nano de la misma. Para esto se hace uso del paquete `rosserial`, permitiendo la administración de la comunicación y el uso de mensajes de ROS de forma nativa con Arduino.
(Consulta la documentación de rosserial para mayor información [aquí](http://wiki.ros.org/rosserial/Tutorials)).

Para instalar estos recursos, ejecuta los siguientes comandos en una terminal:
```
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial
```

Por otro lado, el robot hace uso de un tipo de mensaje en particular para el control de velocidad y dirección. El control del robot se basa en el uso de mensajes tipo Ackermann (Consulta la documentación de este tipo de mensajes [aquí](http://docs.ros.org/en/jade/api/ackermann_msgs/html/index-msg.html)), permitiendo transmitir mensajes con información como:
```
float32 steering_angle
float32 steering_angle_velocity
float32 speed
float32 acceleration
float32 jerk
```
Para hacer uso de estos mensajes, se deberá instalar el paquete `ackermann_msgs`:
```
sudo apt-get install ros-melodic-ackermann-msgs
```


##### Clonar repositorio de GitHub
En una terminal, ejecuta el siguiente comando en el directorio `home`:
```
git clone http://github.com/AldoAguilar/TruckyAVProject
```
Ve al workspace de ROS del proyecto y compila:
```
cd ~/TruckyAVProject/ros/wokspaces/catkin_ws
catkin_make
```

Agrega el archivo de configuración `setup.bash` a la sesión bash de la computadora para que pueda ser cargado cada vez que abras una sesión de terminal nueva:
```
echo "source ~/TruckyAVProject/ros/wokspaces/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Computadora personal
##### Instalación de paquetes de ROS
En una terminal, ejecuta el siguiente comando para instalar el paquete `ackermann_msgs`:
```
sudo apt-get install ros-melodic-ackermann-msgs
```
##### Clonar repositorio de GitHub
En una terminal, ejecuta el siguiente comando en cualquier directorio:
```
git clone http://github.com/AldoAguilar/TruckyAVProject
```

##### Instalación de mensajes personalizados
1. Dirigete a la ubicación en tu computadora donde clonaste el repositorio de GitHub.
2. Dirigete a la ubicación `ros/packages`, dentro deberás encontrar el paquete `trucky_custom_msgs` de ROS con los mensajes personalizados usados para el funcionamiento del robot.
3. Copia la carpeta `trucky_custom_msgs` a la carpeta `src` dentro del `catkin_ws` que utilizas en tu computadora. Esto permitirá que tu computadora reconozca los mensajes personalizados del robot para su utilización.
4. Compila tu `catkin_ws` usando el comando `catkin_make` y utiliza el comando `source devel/setup.bash` para cargar los nuevos recursos.

## Descripción de contenidos
* **packages** : Carpeta con paquetes personalizados necesarios para el funcionamiento y/o comunicación remota con el robot via ROS.
  - **trucky_custom_msgs** : Paquete con los mensajes personalizados usados en el robot.
* **workspaces** : Carpeta con los workspaces de ROS utilizados en el robot.
  - **catkin_ws** : Workspace principal del robot
    - **src/trucky_custom_msgs** : Paquete con los mensajes personalizados usados en el robot.
    - **src/trucky_arduino** : Paquete con los recursos necesarios para la comunicación del robot con el driver de motores con Arduino.
