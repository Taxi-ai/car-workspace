# Taxi-ai car catkin workspace 
> workspace for car contain all package.

## current working nodes 

* camera publish node
* lidar publish node
* lane detection node
* mark detection node
* behavior planning node
* dbw-fsm node "controller"
* servo setter node "controller"

## code structure 
```

* ydlidar "packge for our ydlidar" 
|
* sensors "package to manage all sensors"
|   - msg "contain our custom type messages"
|   - scripts "our python files for publush and subscripe"
|   - launch "launch file to run nodes"
|
* perception "package contain all perception work."
|   - msg "contain our custom type messages"
|   - launch "launch file to run nodes"
|   - src "contain different perception methods"
|   |   - lane-detection
|   |   - mark-detection
|
* planning "pkg contain planning node"
|   - src "contain .cpp files"
|
* fsm-control "controller pkg"
|   - src
|
* launch "contain just launch files to simplify launch systems nodes"
|   - launch
|   |   - all-system.launch " launch all systems nodes"
|   |   - perception.launch " launch camera and mark-detection and lane-detection nodes"
|   |   - planning.launch " launch behavior planning node"
|   |   - controler.launch " launch dbw-fsm and servo-setter nodes"


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
$ roslaunch launch all-system.launch (or other launch files)
```
