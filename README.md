# MLDA 2022 ROS Recruitment Task

- [MLDA 2022 ROS Recruitment Task](#mlda-2022-ros-recruitment-task)
  * [Introduction](#introduction)
  * [Getting Started](#getting-started)
  * [Running nodes](#running-nodes)
      - [Available Nodes](#available-nodes)
    + [Run Nodes Sequences](#run-nodes-sequences)
        * [1. Read Map From File](#1-read-map-from-file)
      - [2. Read Map Using Map Painter](#2-read-map-using-map-painter)
  * [Search Parameters](#search-parameters)
      - [Parameters](#parameters)
      - [Visualization Key](#visualization-key)
  * [Map Painter](#map-painter)
      - [List of keyboard controls](#list-of-keyboard-controls)
  * [RQT Graph](#rqt-graph)
      - [Graph For Map Read Node](#graph-for-map-read-node)
      - [Graph For Map Painter](#graph-for-map-painter)

## Introduction
This is an attempt to implement the Rapidly-exploring Random Tree (RRT) algorithm developed by Lavalle et al, a versitile path planning algorithm applicable to high dimensional space. The implementation also includes the RRT* which is another flavor of the RRT. This is a ROS2 implementation, developed and tested on Ubuntu 20.04 running ROS 2 Foxy. The attempt at the task contains
<br/>

### RRT
![RRT GIF](/imgs/rrt_1.gif)

### RRT*
![RRT* GIF](/imgs/rrt*_2.gif)


Note: The speed of GIF shown above may vary.

## Getting Started
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

## Running nodes
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

### Run Nodes Sequences

1. Start Map Server Node: `ros2 run cpp_rrt_1 map_server_node`
2. Start RRT Planner: `ros2 run py_rrt_1 rrt_planner`
3. **Optional but recommended** Run rviz for visualization: `rviz2` 
    1. Add `Map` Object in rviz from `/map` topic
    2. Add `Path` Object in rviz from `/path` topic
    3. Add `Image` Object in rviz from `/rrt_WIP` topic
    4. Note: The fixed frame in rviz must have name "map"

After this, you have the option to use the `read_image_node` or `map_painter` to load the map into the system.

##### 1. Read Map From File
`ros2 run cpp_rrt_1 read_image_node <absolue path to map file>`

Example:
```
ros2 run cpp_rrt_1 read_image_node ~/Downloads/Maps/map1.png
```
<br/>

#### 2. Read Map Using Map Painter
`ros2 run py_rrt_1 map_painter`  
This will open the map painter.  

Press `o` to open the Open File window.  
This will load the file onto the map painter.  

Press `s` to send the map into the `map_server_node`  

More information and advanced usage of the **map painter** can be found in another section below.  
<br/>
<br/>
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
The example will run vanilla RRT with step size of `20` pixels, maximum node of `10000`, strict steering and no work-in-progress visualization. The parameters will be explained in another section below.

## Search Parameters
To start the search, you need to publish the search parameters to `/start_plan` topic as `cpp_rrt_1/msg/StartPlan` message type. The message type will be explained below.

Note: To run RRT*, set `neighbour_radius` to positive number. Else it is going to run RRT.


RRT Example:
```
ros2 topic pub -1 /start_plan cpp_rrt_1/msg/StartPlan "{step_size: 20, visualize_frequency: 50}"
```

RRT* Example:
```
ros2 topic pub -1 /start_plan cpp_rrt_1/msg/StartPlan "{step_size: 20, max_node_limit: 5000, neighbour_radius: 40, stop_on_found: false, visualization_frequency: 50}"
```


#### Parameters
Name | Type | Defaul Value | Description
-----|------|--------------|------------
`step_size` | `int64` | No Default Value | Max step size of tree in pixels. Recommended value for 500x500 map: `20`
`max_node_limit` | `int64` | `10000` | Number of nodes to insert before stopping the search. 
`max_iteration_limit` | `int64` | `1000000` | Number of iteration before stopping the search. As a fail safe if no nodes can be inserted (no more space). Usually signifies that the tree is trapped. If you increase `max_node_limit` make sure to increase `max_iteration_limit` too.
`goal_bias` | `float64` | `0.05` | Bias to sample goal as random configuration. Recommended in literature: `0.05 - 0.10` or `5% - 10%`.
`neighbour_radius` | `float64` | `0.0` | Neighbour search radius for RRT*. Radius of `0` essentially is RRT. For RRT*, we recommend 2 to 3 times of `step_size`.
`strict` | `bool` | `true` | Strict steering. If `true` the tree can only grow if the growth path 1. hits the max step size, 2. hits the goal node, 3. hits the sampled point. If `false` the growth path will also include points at collision if the collision is less than max step size.
`stop_on_found` | `bool` | `true` | `true` tells the node to stop when a path is found. To run RRT* and see improvement, this needs to be `false`
`visualize_frequency` | `int64` | `0` | To turn on visualization (publish visualization to view in rviz) set this to positive integer. Tells the node to publish the visualization every `visualization_frequency` iteration. Recommended value: `50`
`path` | `bool` | `true` | `true` for path highlighting in visualization (highlight in dark blue)
`reach` | `bool` | `false` | `true` for highlighting the nodes that reach the sampled point (highlight in purple)
`sample` | `bool` | `false` | `true` for highlihting the latest sampled point in yellow

#### Visualization Key
Color | Description
------|------------
Green | Start Node
Red | Goal Node
Light Blue | General Nodes and Edges
Dark Blue | Path Nodes and Edges
Purple | Nodes At Sampled Point
Yellow | Latest Sampled Point

## Map Painter
To open the map painter run: `ros2 run py_rrt_1 map_painter`.
This is a GUI to drawing/editing map. You can use your mouse to draw on the canvas using left mouse button.

#### List of keyboard controls
Key | Description
----|-------------
`d` | Draw Tool / Brush Tool
`e` | Eraser Tool
`y` | Increase brush size
`h` | Decrease brush size
`f` | Set Start Point Tool
`g` | Set Goal Point Tool
`o` | Open map from file
`s` | Send map to `/map_image`
`c` | Save map to disk
`q` | Quit

The user can be in 4 modes, one for each tool. When user is in each tool, the mouse pointer will change depending on the tool.
Tool/Mode | Appearance
----------|------------
Draw Tool | Write Circle
Eraser Tool | Gray Circle
Set Start Point Tool | Green Rectangle
Set Goal Point Tool | Red Rectangle


## RQT Graph

#### Graph For Map Read Node
![RQT Map Read](/imgs/rqt_map_read.png)

#### Graph For Map Painter
![RQT Map Paint](/imgs/rqt_map_paint.png)
