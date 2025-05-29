## PlanSys2 Model

A PlanSys2 v2.0.18 model for the emulator. Developed for ROS2 Jazzy.

Inspiration taken from: 
- https://github.com/PlanSys2/ros2_planning_system_examples
- https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_bt_example
- https://plansys2.github.io/

## Prerequisites:

Install PlanSys2 packages: `sudo apt install ros-jazzy-plansys2-*`

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
     colcon build --packages-select plansys2_model
     ```
3. Source the workspace: `source install/setup.bash` if not already done.
4. Run with `ros2 launch plansys2_model plansys2_model.launch.py`

### In a second terminal:

5. Source ROS workspace with `source /opt/ros/jazzy/setup.bash`
6. Run the PlanSys2 Terminal `ros2 run plansys2_terminal plansys2_terminal`
7. Insert the commands for the problem, see file located at `/launch/commands`.
  

## Example run:

```bash
ros2 launch plansys2_model plansys2_model.launch.py
[INFO] [launch]: All log files can be found below /home/<user>/.ros/log/<log>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [plansys2_node-1]: process started with pid [2364]
[INFO] [gantry_move_action_node-2]: process started with pid [2365]
[INFO] [gantry_calibrate_action_node-3]: process started with pid [2366]
[INFO] [gantry_lock_action_node-4]: process started with pid [2367]
[INFO] [gantry_unlock_action_node-5]: process started with pid [2368]
[INFO] [robot_move_action_node-6]: process started with pid [2369]
[INFO] [robot_mount_action_node-7]: process started with pid [2370]
[INFO] [robot_unmount_action_node-8]: process started with pid [2371]
[robot_move_action_node-6] [INFO] [1748476328.198144687] [robot_move_action_node]: RobotMoveActionNode constructed
[robot_move_action_node-6] [INFO] [1748476328.198215078] [robot_move_action_node]: RobotEmulatorAction registered for 'move_robot'
[gantry_lock_action_node-4] [INFO] [1748476328.198479919] [gantry_lock_action_node]: GantryLockActionNode constructed
[gantry_lock_action_node-4] [INFO] [1748476328.198540761] [gantry_lock_action_node]: GantryEmulatorAction registered for 'lock'
[robot_unmount_action_node-8] [INFO] [1748476328.198547960] [robot_unmount_action_node]: RobotUnmountActionNode constructed
[robot_unmount_action_node-8] [INFO] [1748476328.198607022] [robot_unmount_action_node]: RobotEmulatorAction registered for 'unmount'
[gantry_calibrate_action_node-3] [INFO] [1748476328.213794936] [gantry_calibrate_action_node]: GantryCalibrateActionNode constructed
[gantry_calibrate_action_node-3] [INFO] [1748476328.213886578] [gantry_calibrate_action_node]: GantryEmulatorAction registered for 'calibrate'
[gantry_move_action_node-2] [INFO] [1748476328.217167649] [gantry_move_action_node]: GantryMoveActionNode constructed
[gantry_move_action_node-2] [INFO] [1748476328.217260252] [gantry_move_action_node]: GantryEmulatorAction registered for 'move_gantry'
[robot_mount_action_node-7] [INFO] [1748476328.217421396] [robot_mount_action_node]: RobotMountActionNode constructed
[robot_mount_action_node-7] [INFO] [1748476328.217500668] [robot_mount_action_node]: RobotEmulatorAction registered for 'mount'
[gantry_unlock_action_node-5] [INFO] [1748476328.272059770] [gantry_unlock_action_node]: GantryUnlockActionNode constructed
[gantry_unlock_action_node-5] [INFO] [1748476328.272124852] [gantry_unlock_action_node]: GantryEmulatorAction registered for 'unlock'
[plansys2_node-1] [INFO] [1748476328.382065503] [domain_expert_lc_mngr]: Creating client for service [domain_expert/get_state]
[plansys2_node-1] [INFO] [1748476328.382153775] [domain_expert_lc_mngr]: Creating client for service [domain_expert/change_state]
[plansys2_node-1] [INFO] [1748476328.385606136] [executor_lc_mngr]: Creating client for service [executor/get_state]
[plansys2_node-1] [INFO] [1748476328.385645647] [executor_lc_mngr]: Creating client for service [executor/change_state]
[plansys2_node-1] [INFO] [1748476328.387752829] [planner_lc_mngr]: Creating client for service [planner/get_state]
[plansys2_node-1] [INFO] [1748476328.387790710] [planner_lc_mngr]: Creating client for service [planner/change_state]
[plansys2_node-1] [INFO] [1748476328.389918422] [problem_expert_lc_mngr]: Creating client for service [problem_expert/get_state]
[plansys2_node-1] [INFO] [1748476328.389957803] [problem_expert_lc_mngr]: Creating client for service [problem_expert/change_state]
[plansys2_node-1] [INFO] [1748476328.392473005] [planner]: [planner] Configuring...
[plansys2_node-1] [INFO] [1748476328.393819418] [planner]: Created solver : POPF of type plansys2/POPFPlanSolver
[plansys2_node-1] [INFO] [1748476328.393852519] [planner]: [planner] Solver Timeout 15
[plansys2_node-1] [INFO] [1748476328.395941320] [planner]: [planner] Configured
[plansys2_node-1] [INFO] [1748476328.396115404] [planner_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.396267348] [planner_lc_mngr]: Node planner_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748476328.396354185] [domain_expert]: [domain_expert] Configuring...
[plansys2_node-1] [INFO] [1748476328.398075192] [domain_expert]: Writing domain validation results to /tmp.
[plansys2_node-1] [INFO] [1748476328.562047530] [domain_expert]: [domain_expert] Configured
[plansys2_node-1] [INFO] [1748476328.562195694] [domain_expert_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.562362748] [domain_expert_lc_mngr]: Node domain_expert_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748476328.562449260] [problem_expert]: [problem_expert] Configuring...
[plansys2_node-1] [INFO] [1748476328.562602173] [problem_expert]: [problem_expert] Configured
[plansys2_node-1] [INFO] [1748476328.562669025] [problem_expert_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.562783788] [problem_expert_lc_mngr]: Node problem_expert_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748476328.562886585] [executor]: [executor] Configuring...
[plansys2_node-1] [INFO] [1748476328.639192058] [planner_client]: Planner CLient created with timeout 15
[plansys2_node-1] [INFO] [1748476328.639259449] [executor]: [executor] Configured
[plansys2_node-1] [INFO] [1748476328.639646568] [executor_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.639799192] [executor_lc_mngr]: Node executor_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748476328.639874745] [domain_expert]: [domain_expert] Activating...
[plansys2_node-1] [INFO] [1748476328.639883955] [domain_expert]: [domain_expert] Activated
[plansys2_node-1] [INFO] [1748476328.639928996] [domain_expert_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.640014834] [problem_expert]: [problem_expert] Activating...
[plansys2_node-1] [INFO] [1748476328.640025384] [problem_expert]: [problem_expert] Activated
[plansys2_node-1] [INFO] [1748476328.640073975] [problem_expert_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.640166118] [planner]: [planner] Activating...
[plansys2_node-1] [INFO] [1748476328.640174248] [planner]: [planner] Activated
[plansys2_node-1] [INFO] [1748476328.640215729] [planner_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.640288955] [executor]: [executor] Activating...
[plansys2_node-1] [INFO] [1748476328.640298735] [executor]: [executor] Activated
[plansys2_node-1] [INFO] [1748476328.640343585] [executor_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748476328.640476130] [domain_expert_lc_mngr]: Node domain_expert_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748476328.640593043] [problem_expert_lc_mngr]: Node problem_expert_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748476328.640696294] [planner_lc_mngr]: Node planner_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748476328.640800097] [executor_lc_mngr]: Node executor_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748476336.399153323] [planner]: Writing planning results to /tmp.
[plansys2_node-1] [INFO] [1748476336.399336957] [planner]: [planner-popf] called with timeout 15.000000 seconds
[plansys2_node-1] [INFO] [1748476336.542029575] [planner]: Writing planning results to /tmp.
[plansys2_node-1] [INFO] [1748476336.542196929] [planner]: [planner-popf] called with timeout 15.000000 seconds
[plansys2_node-1] [INFO] [1748476336.690011442] [executor]: Action calibrate timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748476336.692540996] [executor]: Action lock timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748476336.695046897] [executor]: Action move_robot timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748476336.697575924] [executor]: Action mount timeout percentage -1.000000
[plansys2_node-1] getExpr: Error parsing expresion [(and (calibrated gantry1))]
[plansys2_node-1] getExpr: Error parsing expresion [(calibrated gantry1)]
[plansys2_node-1] getExpr: Error parsing expresion [(and (locked gantry1))]
[plansys2_node-1] getExpr: Error parsing expresion [(locked gantry1)]
[plansys2_node-1] getExpr: Error parsing expresion [(and (at_robot robot1 b1))]
[plansys2_node-1] getExpr: Error parsing expresion [(at_robot robot1 b1)]
[plansys2_node-1] getExpr: Error parsing expresion [(and (mounted robot1 tool1))]
[plansys2_node-1] getExpr: Error parsing expresion [(mounted robot1 tool1)]
[plansys2_node-1] [WARN] [1748476336.704240103] [rcl.logging_rosout]: Publisher already registered for node name: 'domain_expert_client'. If this is due to multiple nodes with the same name then all logs for the logger named 'domain_expert_client' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[plansys2_node-1] [WARN] [1748476336.733577665] [rcl.logging_rosout]: Publisher already registered for node name: 'problem_expert_client'. If this is due to multiple nodes with the same name then all logs for the logger named 'problem_expert_client' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[gantry_calibrate_action_node-3] [INFO] [1748476336.875845351] [gantry_calibrate_action_node]: GantryCalibrateActionNode::do_work() called
[gantry_calibrate_action_node-3] [INFO] [1748476336.875880742] [gantry_calibrate_action_node]: Command: calibrate, Position: gantry1
[gantry_calibrate_action_node-3] [INFO] [1748476336.875980684] [gantry_calibrate_action_node]: Sending request to gantry emulator service
[gantry_lock_action_node-4] [INFO] [1748476336.876031415] [gantry_lock_action_node]: GantryLockActionNode::do_work() called
[gantry_lock_action_node-4] [INFO] [1748476336.876051766] [gantry_lock_action_node]: Command: lock, Position: gantry1
[gantry_lock_action_node-4] [INFO] [1748476336.876151658] [gantry_lock_action_node]: Sending request to gantry emulator service
[robot_move_action_node-6] [INFO] [1748476336.876645005] [robot_move_action_node]: RobotMoveActionNode::do_work() called
[robot_move_action_node-6] [INFO] [1748476336.876679616] [robot_move_action_node]: Command: move_robot, Position: robot1
[robot_move_action_node-6] [INFO] [1748476336.876778378] [robot_move_action_node]: Sending request to robot emulator service
[gantry_lock_action_node-4] terminate called after throwing an instance of 'std::runtime_error'
[gantry_calibrate_action_node-3] terminate called after throwing an instance of 'std::runtime_error'
[robot_move_action_node-6] terminate called after throwing an instance of 'std::runtime_error'
[robot_mount_action_node-7] [INFO] [1748476336.876953728] [robot_mount_action_node]: RobotMountActionNode::do_work() called
[robot_mount_action_node-7] [INFO] [1748476336.876973258] [robot_mount_action_node]: Command: mount, Position: robot1
[robot_mount_action_node-7] [INFO] [1748476336.877099791] [robot_mount_action_node]: Sending request to robot emulator service
[gantry_calibrate_action_node-3]   what():  Node '/gantry_calibrate_action_node' has already been added to an executor.
[gantry_lock_action_node-4]   what():  Node '/gantry_lock_action_node' has already been added to an executor.
[robot_move_action_node-6]   what():  Node '/robot_move_action_node' has already been added to an executor.
[robot_mount_action_node-7] terminate called after throwing an instance of 'std::runtime_error'
[robot_mount_action_node-7]   what():  Node '/robot_mount_action_node' has already been added to an executor.
```