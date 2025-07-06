# engg02

### Necessario ter o kinova-ros package funcionando. Esse git funciona so como uma mascara para as funcionalidades do kinova-ros (criei ele pra usar especificamente o Jaco porque o git do kinova oferece suporte para mais bracos roboticos)

#### Instale os seguites repositorios:
```
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-ros-controllers
sudo apt-get install ros-noetic-trac-ik-kinematics-plugin
sudo apt-get install ros-noetic-moveit
sudo apt-get install ros-noetic-trac-ik
```
#### Se quiser testar o jaco com o launcher do robo em cima da mesa:
```
roslaunch jaco_myo jaco.launch
```
```
roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch
```

#### Se quiser testar o jaco no launcher original do kinova
```
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300
```
```
roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch
```
