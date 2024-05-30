# Navegación autónoma desdde ROS+PX4

Este framework fue desarrollado por los desarrolladores del código abierto de RTAB Map, el cual usa sensores de escaneo para hacer un análisis de imágenes de manera incremental, y con el algoritmo de cierre de bucle incremental actualiza su posición en tiempo real, dando por si solo una navegación muy eficiente en robótica, especialmente en las aplicaciones de drones debido a que da una localización precisa en las tres dimensiones de vuelo, algo que no ocurre con otros tipos de navegación.
Ahora se explicará el proceso de instalación de este framework, primero se dan los requisitos de la máquina antes de la instalación del framework.
•	Ubuntu 18.04 ó Ubuntu 20.04.
•	ROS Noetic ó Melodic versión completa.
•	Paquetes de ROS como octomap, mavros, rtabmap y vision_msgs.
•	Framework de PX4-Autopilot.
•	Gazebo.
Una vez cumpliendo los requisitos mínimos, y se instalan los paquetes requeridos mostrados en los pasados informes, se procede a instalar las dependencias de ROS para nuestra máquina de ROS Noetic.

 ````
sudo apt install ros-noetic-octomap* && sudo apt install ros-noetic-mavros* 
sudo apt install ros-noetic-vision-msgs
````

Se crea el workspace para el repositorio.

 ````
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/ProyectoDagger/Navegacion_autonoma_px4_ros.git

````

Ahora se crea el espacio de trabajo con catkin.
 ````
cd catkin_ws
catkin build
````
## Instalación PX4-Autopilot

Ahora se instala el código de PX4 Autopilot
 ````
cd ~

git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./Tools/setup/ubuntu.sh
````
Junto a este, se instala Gazebo
 ````
cd ~
curl -sSL http://get.gazebosim.org | sh
````
Finalmente, se pone en la carpeta raíz los siguientes parámetros
 ````
sudo nano ~./bashrc

sudo /opt/ros/noetic/setup.bash
source PX4-Autopilot/Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)PX4-Autopilot/Tools/sitl_gazebo
````
Finalmente, se guarda el archivo con ctrl+X y se procede a iniciar la simulación.
Para ejecutar la simulación, seguimos las siguientes líneas:
 ````
cd catkin_ws
source devel/setup.bash
roslaunch rtabmap_drone_example gazebo.launch
roslaunch rtabmap_drone_example slam.launch
roslaunch rtabmap_drone_example rviz.launch
rosrun rtabmap_drone_example offboard  
````
![image](https://github.com/ProyectoDAGGER/Navegacion_autonoma_px4_ros/assets/163484218/4636c046-a5da-485d-913e-5a92c3127b3a)
