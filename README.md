# DRA-MPPI Planner Workspace (VSCode Docker)
This repository provides a complete VSCode docker environment for running dynamic risk-aware mppi for mobile robots navigation in crowds in ROS1.
You can find more information on the [paper's website](https://autonomousrobots.nl/paper_websites/dra-mppi).


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
## Contributing

Contributions are welcome. Please submit a pull request.

## Cite

If you find this code useful, please cite:
```bash
@ARTICLE{dra-mppi,
  author={Trevisan, Elia, and Mustafa, Khaled, and Notten, Godert, and Wang, Xinwei, and Alonso-Mora, Javier},
  conference={IEEE Robotics and Automation Letters}, 
  title={Dynamic Risk-Aware MPPI for Mobile Robots in Crowds via Efficient Monte Carlo Approximations}, 
  year={2025},
  pages={313-320},
  doi={10.1109/IROS60139.2025.11246822}}
```

