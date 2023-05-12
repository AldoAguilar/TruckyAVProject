# Trucky AV Project Configuración en ROS

Este directorio incluye los recursos necesarios para el control del robot "Trucky" con ROS, permitiendo el control de actuadores y lectura de sensores así como el intercambio de mensajes.

## Tabla de Contenidos

* [Descripción de contenidos](#descripción-de-contenidos)
  - [trucky_custom_msgs](#trucky_custom_msgs)
  - [trucky_ws](#trucky_ws)
* [Instalación](#instalación)
  - [Prerrequisitos de hardware](#prerrequisitos-de-hardware)
  - [Prerrequisitos de software](#prerrequisitos-de-software)
  - [Configuración de software](#configuración-de-software)

## Descripción de contenidos
### trucky_custom_msgs
Es un paquete con los mensajes personalizados usados en el robot. El usuario deberá instalar este paquete en su computadora para poder monitorear y enviar mensajes al robot de forma remota.
Los mensajes definidos en este paquete incluyen:
* **ActuatorsState.msg**
```
int64 servo_pwm_high_time  // Servo Motor PWM High Time in microseconds
int64 motor_pwm_high_time  // BLDC Motor PWM High Time in microseconds
string output_mode         // Robot operating mode "MANUAL" or "AUTO"
```
Este mensaje incluye infromación de las señales de control suministradas a los actuadores del robot en términos del tiempo en alto de las señales PWM correspondientes. Así como del modo de operación actual del robot `"MANUAL"` o `"AUTO"`. Se entiende que el robot se encuentra operando en modo `"MANUAL"` cuando el usuario esta controlando el robot directamente con el controlador de RF. El robot opera en modo `"AUTO"` cuando los actuadores del robot son controlados de forma autónoma por el robot haciendo uso del driver de motores.

* **PIDGains.msg**
```
float32 kp 
float32 kd
float32 ki
```
Mensaje usado para la configuración de las ganancias PID para el control de velocidad del robot. Estos mensajes permiten la configuración manual de dichos parámetros enviandolos a la placa "Trucky Driver Board" para ajustar el comportamiento del sistema de control de velocidad.

### trucky_ws

Workspace principal del robot, en este se almacenan los paquetes necesarios para el funcionamiento del robot.
Los paquetes contenidos en este workspace incluyen:
* **trucky_custom_msgs** : Paquete con los mensajes personalizados usados en el robot. Consulta el apartado [trucky_custom_msgs](#trucky_custom_msgs) para más información.
* **trucky_arduino** : Paquete con los recursos necesarios para la comunicación del robot con el driver de motores con Arduino, permite:
  + Calibrar los actuadores del robot, obteniendo los rangos de operación de las señales de PWM, así como caracterizar el servo motor para el control de dirección del robot.
  + Actualizar y cargar el firmware de los microcontroladores (Arduino Actuators y Arduino Sensors).
  + Establecer la comunicación con el driver de motores y habilitar los tópicos necesarios para el control del robot. 
  + Cambiar el modo de operación del robot, modificando el modo de control del mismo con base en:
    1. El tiempo en alto de las señales de PWM de los actuadores haciendo uso del mensaje **ActuatorsState.msg**.
    2. Comandos tipo ackermann con la velocidad lineal y dirección deseadas usando mensajes tipo **AckermannDrive.msg**
  + Cambiar el modo de monitoreo del robot, modificando el tipo de mensajes de monitoreo de estado del robot usando mensajes como:
    1. La velocidad angular en las ruedas del robot  y el ángulo de dirección de las ruedas forntales .
    2. El estado actual de los actuadores en términos de las señales PWM suministrados en los mismos y la velocidad lineal actual del robot.
    
Consulta más infomración a cerca de estos paquetes [aquí](workspaces/trucky_ws/src/trucky_ws). 

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

Por otro lado, el robot hace uso de un tipo de mensaje en particular para el control de velocidad y dirección. El control del robot se basa en el uso de mensajes tipo **AckermannDrive.msg** (Consulta la documentación de este tipo de mensajes [aquí](http://docs.ros.org/en/jade/api/ackermann_msgs/html/index-msg.html)), permitiendo transmitir mensajes con información como:
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

Valida la correcta instalación de los mensajes ejecutando el comando:

```
rosmsg package ackermann_msgs
```

##### Clonar repositorio de GitHub
En una terminal, ejecuta el siguiente comando en el directorio `home`:
```
cd ~
git clone http://github.com/AldoAguilar/TruckyAVProject
```

Mueve el contenido del workspace del proyecto al directotio `home`, puedes hacerlo manualmente o usando comandos de terminal:

```
mv ~/TruckyAVProject/ros/wokspaces/trucky_ws ~
```

Ve al workspace de ROS del proyecto y compila:
```
cd ~/trucky_ws
catkin_make
source devel/setup.bash
```

Agrega el archivo de configuración `setup.bash` a la sesión bash de la computadora para que pueda ser cargado cada vez que abras una sesión de terminal nueva:
```
echo "source ~/trucky_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Computadora personal
##### Instalación de paquetes de ROS
En una terminal, ejecuta el siguiente comando para instalar el paquete `ackermann_msgs`:
```
sudo apt-get install ros-melodic-ackermann-msgs
```
Valida la correcta instalación de los mensajes ejecutando el comando:

```
rosmsg package ackermann_msgs
```

##### Clonar repositorio de GitHub
En una terminal, ejecuta el siguiente comando en cualquier directorio:
```
git clone http://github.com/AldoAguilar/TruckyAVProject
```

##### Instalación de mensajes personalizados
1. Dirigete a la ubicación en tu computadora donde clonaste el repositorio de GitHub.
2. Dirigete a la ubicación `ros/packages`, dentro deberás encontrar el paquete `trucky_custom_msgs` de ROS con los mensajes personalizados usados para el funcionamiento del robot.
3. Copia la carpeta `trucky_custom_msgs` a la carpeta `src` dentro del workspace de ROS (P.ej: `catkin_ws`) que utilizas en tu computadora. Esto permitirá que tu PC reconozca los mensajes personalizados del robot para su utilización.
4. Compila tu workspace usando el comando `catkin_make` y carga los nuevos recursos:
```
source devel/setup.bash
```
6. Valida la correcta instalación de los mensajes ejecutando el comando:
```
rosmsg package trucky_custom_msgs
```
