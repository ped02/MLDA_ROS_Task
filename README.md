# MLDA 2022 ROS Recruitment Task


### Introduction
This is an attempt to implement the Rapidly-exploring Random Tree (RRT) algorithm developed by Lavalle et al, a versitile path planning algorithm applicable to high dimensional space. The implementation also includes the RRT* which is another flavor of the RRT. This is a ROS2 implementation, developed and tested on Ubuntu 20.04 running ROS 2 Foxy. The attempt at the task contains
<br/>
### Getting Started
Firstly, you need to install ROS 2 Foxy Desktop. You can follow [this](https://docs.ros.org/en/foxy/Installation.html) instruction. Make sure you insall the Desktop as we need rviz.

You also need install the Python dependencies.
```
python3 -m pip install opencv-contrib-python numpy tk
```


Next, create a directory for the ROS 2 workspace.
Example: 

```
mkdir ~/mlda_ws
```

Place the `src` folder inside the newly created directory.

Source ROS 2. This depends on how you installed global ROS 2. We recommend to put this in the `.bashrc` as you need to source ROS 2 in every terminal you are going to run `ros2` and ROS 2 related commands.
Example for normal installation:
```
source /opt/ros/foxy/setup.bash
```

Install ROS 2 project dependencies
```
cd ~/mlda_ws
rosdep -i install --from-path src --rosdistro foxy -y
```

Build the ROS 2 project
```
cd ~/mlda_ws
concol build
```

### Running nodes
To run the nodes, you need to use `ros2 run <package_name> <node_name>`.

Available Nodes
Node Name | Package | Description
----------|---------|------------
read_image_node|cpp_rrt_1|Read map image from file, convert to gray scale and publish to `/map_image` topic for `map_server_node`.
map_server_node|cpp_rrt_1|Subscribes to `/map_image` and convert the map received to `OccupancyGrid` message and publish onto `/map` topic.
rrt_planner|py_rrt_1|Planner Node - subscribes to `/map` topic to get map, subscribes to `/start_point` and `/goal_point` for starting and goal positions for the search, subscribes to `/start_plan` to start plan with specified parameters from the message.
map_painter|py_rrt_1|Map Painter - GUI for painting maps - painting walls and specify start and goal positions. Can be used to load and save maps from `.png` and `.pgm` file. Can also send the map and start and goal positions to `rrt_planner` directly.





