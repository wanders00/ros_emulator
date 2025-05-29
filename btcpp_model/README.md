## BehaviorTree.CPP Model

A BehaviorTree.CPP v4.6 model for the emulator. Developed for ROS2 Jazzy.

Inspiration taken from: 
- https://github.com/BehaviorTree/btcpp_sample
- https://github.com/BehaviorTree/BehaviorTree.ROS2
- https://www.behaviortree.dev/docs/Intro

## Prerequisites:

Install BehaviorTree.CPP package: `sudo apt install ros-jazzy-behaviortree-cpp`

## How to run:

1. Source ROS workspace with `source /opt/ros/jazzy/setup.bash`
2. Build the package(s):
   - **Option 1:** Build all packages in the workspace:
     ```bash
     colcon build
     ```
   - **Option 2:** Build only this package:
     ```bash
     source install/setup.bash
     colcon build --packages-select btcpp_model
     ```
3. Source the workspace: `source install/setup.bash` if not already done.
4. Run with `ros2 launch btcpp_model btcpp_model.launch.py`

## Example run:

```bash
ros2 launch btcpp_model btcpp_model.launch.py
[INFO] [launch]: All log files can be found below /home/<user>/.ros/log/<log>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [btcpp_model-1]: process started with pid [75232]
[btcpp_model-1] [INFO] [1748269230.101938908] [btcpp_model]: Node initialization started.
[btcpp_model-1] [INFO] [1748269230.292100112] [btcpp_model]: Node created.
[btcpp_model-1] [INFO] [1748269230.292478246] [btcpp_model]: Loading behavior tree from: /home/<user>/ros_emulator/install/btcpp_model/share/btcpp_model/behavior_trees/example_tree.xml
[btcpp_model-1] [INFO] [1748269230.292668331] [btcpp_model]: Registering RobotEmulatorAction node: RobotEmulatorAction
[btcpp_model-1] [INFO] [1748269230.292699363] [btcpp_model]: RobotEmulatorAction constructed: RobotEmulatorAction
[btcpp_model-1] [INFO] [1748269230.293528003] [btcpp_model]: Registering GantryEmulatorAction node: GantryEmulatorAction
[btcpp_model-1] [INFO] [1748269230.293564843] [btcpp_model]: GantryEmulatorAction constructed: GantryEmulatorAction
[btcpp_model-1] [INFO] [1748269230.294071637] [btcpp_model]: Registering RobotEmulatorAction node: RobotEmulatorAction
[btcpp_model-1] [INFO] [1748269230.294101008] [btcpp_model]: RobotEmulatorAction constructed: RobotEmulatorAction
[btcpp_model-1] [INFO] [1748269230.294736226] [btcpp_model]: Ticking the behavior tree once.
[btcpp_model-1] [INFO] [1748269230.294774012] [btcpp_model]: RobotEmulatorAction::tick() called
[btcpp_model-1] [INFO] [1748269230.294783329] [btcpp_model]: Command: mount, Position: tool1
[btcpp_model-1] [INFO] [1748269230.294804901] [btcpp_model]: Sending request to robot emulator service
[btcpp_model-1] [INFO] [1748269230.296651892] [btcpp_model]: RobotEmulatorAction: Service response: SUCCESS
[btcpp_model-1] [INFO] [1748269230.296691306] [btcpp_model]: GantryEmulatorAction::tick() called
[btcpp_model-1] [INFO] [1748269230.296699809] [btcpp_model]: Command: move, Position: A1
[btcpp_model-1] [INFO] [1748269230.296708808] [btcpp_model]: Sending request to gantry emulator service
[btcpp_model-1] [INFO] [1748269230.298329462] [btcpp_model]: GantryEmulatorAction: Service response: SUCCESS
[btcpp_model-1] [INFO] [1748269230.298376301] [btcpp_model]: RobotEmulatorAction::tick() called
[btcpp_model-1] [INFO] [1748269230.298386157] [btcpp_model]: Command: unmount, Position: tool1
[btcpp_model-1] [INFO] [1748269230.298396883] [btcpp_model]: Sending request to robot emulator service
[btcpp_model-1] [INFO] [1748269230.299967548] [btcpp_model]: RobotEmulatorAction: Service response: SUCCESS
[btcpp_model-1] [INFO] [1748269230.300052833] [btcpp_model]: Shutting down node.
[INFO] [btcpp_model-1]: process has finished cleanly [pid 75232]
```