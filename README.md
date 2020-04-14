# Taxi-ai car catkin workspace 
> workspace for car contain all package.

## current working nodes 

* camera publish node
* lidar publish node

## TODO
* add IMU sensor node
* try to use sensor_msgs/Image message type in camera node

## code structure 
```

- ydlidar "packge for our ydlidar" 
- sensors "package to manage all sensors"
|   - msg "contain our custom type messages"
|   - scripts "our python files for publush and subscripe"
|   - launch "launch file to run nodes"

- README.md
```

## Installation

OS X & Linux:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:

```sh
$ cd ~/catkin_ws/src
$ git clone clone https://github.com/Taxi-ai/car-workspace.git
```
install dep

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

chmod for all scripts to be to be executable.  

then 

```sh
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch sensors all-sensors.launch (or other launch files)
```
