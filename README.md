# DRA-MPPI Planner Workspace (VSCode Docker)
This repository provides a complete VSCode docker environment for running dynamic risk-aware mppi for mobile robots navigation in crowds in ROS1.

### Container
In VSCode install these extensions:

- https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers
- https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker

Clone this repository. In VSCode: `open_folder` -> open `dra_mppi`. It will show a pop-up: click `Reopen in container`. It will build the container. This will take some time.

### Usage
To run the planner, build the ros packages first by running
```bash
catkin build
source devel/setup.bash
roslaunch mpc_planner_jackalsimulator ros1_jackalsimulator.launch
```


