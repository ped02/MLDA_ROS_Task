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
To run the nodes, you need to source the project 
```
cd ~/mlda_ws
. install/setup.sh
```

Then use `ros2 run <package_name> <node_name>`. 

#### Available Nodes
Node Name | Package | Description
----------|---------|------------
`read_image_node`|`cpp_rrt_1`|Read map image from file, convert to gray scale and publish to `/map_image` topic for `map_server_node`.
`map_server_node`|`cpp_rrt_1`|Subscribes to `/map_image` and convert the map received to `OccupancyGrid` message and publish onto `/map` topic.
`rrt_planner`|`py_rrt_1`|Planner Node - subscribes to `/map` topic to get map, subscribes to `/start_point` and `/goal_point` for starting and goal positions for the search, subscribes to `/start_plan` to start plan with specified parameters from the message. The node publishes work-in-progress search onto `/rrt_WIP` topic and publishes path to `/path` topic once the search is done.
`map_painter`|`py_rrt_1`|Map Painter - GUI for painting maps - painting walls and specify start and goal positions. Can be used to load and save maps from `.png` and `.pgm` file. Can also send the map and start and goal positions to `rrt_planner` directly.

Note: There are also nodes used for internal testing which is not listed here.

#### Run Nodes Sequences

1. Start Map Server Node: `ros2 run cpp_rrt_1 map_server_node`
2. Start RRT Planner: `ros2 run py_rrt_1 rrt_planner`
3. **Optional but recommended** Run rviz for visualization: `rviz2` 
    1. Add `Map` Object in rviz from `/map` topic
    2. Add `Path` Object in rviz from `/path` topic
    3. Add `Image` Object in rviz from `/rrt_WIP` topic
    4. Note: The fixed frame in rviz must have name "map"

After this, you have the option to use the `read_image_node` or `map_painter` to load the map into the system.

##### Read Map From File
`ros2 run cpp_rrt_1 read_image_node <absolue path to map file>`

Example:
```
ros2 run cpp_rrt_1 read_image_node ~/Downloads/Maps/map1.png
```

#### Read Map Using Map Painter
`ros2 run py_rrt_1 map_painter`  
This will open the map painter.  

Press `o` to open the Open File window.  
This will load the file onto the map painter.  

Press `s` to send the map into the `map_server_node`  

More information and advanced usage of the **map painter** can be found in another section below.  

After loading the map into the system, you need to specify the start and goal points by publishing onto the `/start_point` and `/goal_point` topic as `geometry_msgs/msg/Point` message type.
Example:
```
ros2 topic pub -1 /start_point geometry_msgs/msg/Point "{x: 50, y: 50, z: 0}"
ros2 topic pub -1 /goal_point geometry_msgs/msg/Point "{x: 200, y: 370, z: 0}"
```

After specifying the start and goal points. You can start the search by publishing the search parameters onto the `/start_plan` topic as `cpp_rrt_1/msg/StartPlan` message type.
Example:
```
ros2 topic pub -1 /start_plan cpp_rrt_1/msg/StartPlan "{step_size: 20}"
```
The example will run vanilla RRT with step size of 20 pixels, maximum node of 10000, strict steering and no work-in-progress visualization. The parameters will be explained in another section below.
