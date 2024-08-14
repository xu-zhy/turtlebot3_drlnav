# Mobile Robot DRL Navigation

### A ROS2 framework for DRL autonomous navigation on mobile robots with LiDAR.
<p float="left">
 <img src="media/simulation.gif" width="400">
 <img src="media/physical_demo.gif" width="216" alt="physical_demo.gif" />
</p>


# **Table of contents**
* [Installation](#installation)
  * [Manual Installation](#manual-installation)
* [Training and Testing](#training-and-testing)
  * [Command](#commands)
  * [Trained Models](#trained-models)
  * [Worlds](#worlds)
    * [BARN Dataset](#barn-dataset)
    * [Other Worlds](#other-worlds)


# **Manual Installation**

Docker Installation and detailed intallation guide can refer to the original [repository](https://github.com/tomasvr/turtlebot3_drlnav).

## **Dependencies**

*   Ubuntu 20.04 LTS (Focal Fossa) [download](https://releases.ubuntu.com/20.04)
*   ROS2 Foxy Fitzroy
*   Gazebo (Version 11.0)
*   PyTorch (Version: 1.10.0)

## **Downloading the code base and building**

First, make sure you have the `turtlebot3-description` package by running:
```
sudo apt-get install ros-foxy-turtlebot3-description
```

Open a terminal in the desired location for the new workspace. Clone the repository using:
```
git clone https://github.com/tomasvr/turtlebot3_drlnav.git
```

`cd` into the directory and make sure you are on the main branch
```
cd turtlebot3_drlnav
git checkout main
```

Next, install the correct rosdep tool
```
sudo apt install python3-rosdep2
```

Then initialize rosdep by running
```
rosdep update
```

Now we can use rosdep to install all ROS packages needed by our repository
```
rosdep install -i --from-path src --rosdistro foxy -y
```

Now that we have all of the packages in place it is time to build the repository. First update your package list
```
sudo apt update
```

Then install the build tool **colcon** which we will use to build our ROS2 package
```
sudo apt install python3-colcon-common-extensions
```

Next, it's time to actually build the repository code!
```
colcon build
```
After colcon has finished building source the repository
```
source install/setup.bash
```

The last thing we need to do before running the code is add a few lines to our `~/.bashrc` file so that they are automatically executed whenever we open a new terminal. Add the following lines at the end of your `~/.bashrc` file and **replace ~/path/to/turtlebot3_drlnav/repo with the path where you cloned the repository. (e.g. ~/turtlebot3_drlnav)**
```
# ROS2 domain id for network communication, machines with the same ID will receive each others' messages
export ROS_DOMAIN_ID=1

# Fill in the path to where you cloned the turtlebot3_drlnav repo
WORKSPACE_DIR=~/path/to/turtlebot3_drlnav
export DRLNAV_BASE_PATH=$WORKSPACE_DIR

# Source the workspace
source $WORKSPACE_DIR/install/setup.bash

# Allow gazebo to find our turtlebot3 models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$WORKSPACE_DIR/src/turtlebot3_simulations/turtlebot3_gazebo/models

# Select which turtlebot3 model we will be using (default: burger, waffle, waffle_pi)
export TURTLEBOT3_MODEL=burger

# Allow Gazebo to find the plugin for moving the obstacles
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$WORKSPACE_DIR/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib
```

For more detailed instructions on ros workspaces check [this guide](https://automaticaddison.com/how-to-create-a-workspace-ros-2-foxy-fitzroy/).

**Note: Always make sure to first run ```source install/setup.bash``` or open a fresh terminal after building with `colcon build`.**


# Training and Testing
## Commands
Open up four different terminals however you like (I recommended using `terminator` or `tmux` for multi-tab). In the first terminal run
```
ros2 launch turtlebot3_gazebo turtlebot3_drl_stage4.launch.py
```
In a second terminal run
```
ros2 run turtlebot3_drl gazebo_goals
```

In a third terminal run
```
ros2 run turtlebot3_drl environment
```

In the fourth terminal run the ddpg agent
```
# train
ros2 run turtlebot3_drl train_agent ddpg "examples/ddpg_0_stage9" 8000
# test
ros2 run turtlebot3_drl test_agent ddpg "examples/ddpg_0_stage9" 8000
```

## Trained Models

* DQN: 2900 episodes
* DDPG: 8000 episodes
* TD3: 7400 episodes

## Worlds

In `turtlebot3_drl_stage12.launch`, you can use the `world_file_name` argument to change the simulation world.
```
ros2 launch turtlebot3_gazebo turtlebot3_drl_stage12.launch.py world_file_name:=wall1.model
```

## BARN Dataset

Testing TD3 in [BARN dataset](https://www.cs.utexas.edu/~xiao/BARN/BARN.html)


### Other Worlds

* apartment.world
* wall1.world