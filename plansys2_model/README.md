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
[INFO] [plansys2_node-1]: process started with pid [92294]
[INFO] [gantry_move_action_node-2]: process started with pid [92295]
[INFO] [gantry_calibrate_action_node-3]: process started with pid [92296]
[INFO] [gantry_lock_action_node-4]: process started with pid [92297]
[INFO] [gantry_unlock_action_node-5]: process started with pid [92298]
[INFO] [robot_move_action_node-6]: process started with pid [92299]
[INFO] [robot_mount_action_node-7]: process started with pid [92300]
[INFO] [robot_unmount_action_node-8]: process started with pid [92301]
[gantry_unlock_action_node-5] [INFO] [1748541896.186759899] [gantry_unlock_action_node]: GantryUnlockActionNode constructed
[gantry_move_action_node-2] [INFO] [1748541896.188158381] [gantry_move_action_node]: GantryMoveActionNode constructed
[gantry_calibrate_action_node-3] [INFO] [1748541896.189501567] [gantry_calibrate_action_node]: GantryCalibrateActionNode constructed
[gantry_lock_action_node-4] [INFO] [1748541896.197051460] [gantry_lock_action_node]: GantryLockActionNode constructed
[robot_mount_action_node-7] [INFO] [1748541896.199807639] [robot_mount_action_node]: RobotMountActionNode constructed
[robot_unmount_action_node-8] [INFO] [1748541896.206211943] [robot_unmount_action_node]: RobotUnmountActionNode constructed
[robot_move_action_node-6] [INFO] [1748541896.208890236] [robot_move_action_node]: RobotMoveActionNode constructed
[plansys2_node-1] [INFO] [1748541896.344256360] [domain_expert_lc_mngr]: Creating client for service [domain_expert/get_state]
[plansys2_node-1] [INFO] [1748541896.344377532] [domain_expert_lc_mngr]: Creating client for service [domain_expert/change_state]
[plansys2_node-1] [INFO] [1748541896.347393594] [executor_lc_mngr]: Creating client for service [executor/get_state]
[plansys2_node-1] [INFO] [1748541896.347461855] [executor_lc_mngr]: Creating client for service [executor/change_state]
[plansys2_node-1] [INFO] [1748541896.349974884] [planner_lc_mngr]: Creating client for service [planner/get_state]
[plansys2_node-1] [INFO] [1748541896.350019474] [planner_lc_mngr]: Creating client for service [planner/change_state]
[plansys2_node-1] [INFO] [1748541896.352334550] [problem_expert_lc_mngr]: Creating client for service [problem_expert/get_state]
[plansys2_node-1] [INFO] [1748541896.352377141] [problem_expert_lc_mngr]: Creating client for service [problem_expert/change_state]
[plansys2_node-1] [INFO] [1748541896.355228010] [planner]: [planner] Configuring...
[plansys2_node-1] [INFO] [1748541896.356017102] [planner]: Created solver : POPF of type plansys2/POPFPlanSolver
[plansys2_node-1] [INFO] [1748541896.356047033] [planner]: [planner] Solver Timeout 15
[plansys2_node-1] [INFO] [1748541896.358060285] [planner]: [planner] Configured
[plansys2_node-1] [INFO] [1748541896.358205053] [planner_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.358353540] [planner_lc_mngr]: Node planner_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748541896.358457267] [domain_expert]: [domain_expert] Configuring...
[plansys2_node-1] [INFO] [1748541896.360579444] [domain_expert]: Writing domain validation results to /tmp.
[plansys2_node-1] [INFO] [1748541896.542457556] [domain_expert]: [domain_expert] Configured
[plansys2_node-1] [INFO] [1748541896.542695220] [domain_expert_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.542914439] [domain_expert_lc_mngr]: Node domain_expert_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748541896.543024255] [problem_expert]: [problem_expert] Configuring...
[plansys2_node-1] [INFO] [1748541896.543207078] [problem_expert]: [problem_expert] Configured
[plansys2_node-1] [INFO] [1748541896.543300110] [problem_expert_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.543461172] [problem_expert_lc_mngr]: Node problem_expert_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748541896.543569083] [executor]: [executor] Configuring...
[plansys2_node-1] [INFO] [1748541896.629675116] [planner_client]: Planner CLient created with timeout 15
[plansys2_node-1] [INFO] [1748541896.629725827] [executor]: [executor] Configured
[plansys2_node-1] [INFO] [1748541896.630011256] [executor_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.630187428] [executor_lc_mngr]: Node executor_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748541896.630260710] [domain_expert]: [domain_expert] Activating...
[plansys2_node-1] [INFO] [1748541896.630271690] [domain_expert]: [domain_expert] Activated
[plansys2_node-1] [INFO] [1748541896.630318685] [domain_expert_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.630405513] [problem_expert]: [problem_expert] Activating...
[plansys2_node-1] [INFO] [1748541896.630418943] [problem_expert]: [problem_expert] Activated
[plansys2_node-1] [INFO] [1748541896.630467733] [problem_expert_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.630570765] [planner]: [planner] Activating...
[plansys2_node-1] [INFO] [1748541896.630581115] [planner]: [planner] Activated
[plansys2_node-1] [INFO] [1748541896.630628286] [planner_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.630720187] [executor]: [executor] Activating...
[plansys2_node-1] [INFO] [1748541896.630731537] [executor]: [executor] Activated
[plansys2_node-1] [INFO] [1748541896.630776723] [executor_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748541896.630915480] [domain_expert_lc_mngr]: Node domain_expert_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748541896.631052813] [problem_expert_lc_mngr]: Node problem_expert_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748541896.631170654] [planner_lc_mngr]: Node planner_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748541896.631322807] [executor_lc_mngr]: Node executor_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748541905.791947461] [planner]: Writing planning results to /tmp.
[plansys2_node-1] [INFO] [1748541905.792151894] [planner]: [planner-popf] called with timeout 15.000000 seconds
[plansys2_node-1] [INFO] [1748541905.968538100] [planner]: Writing planning results to /tmp.
[plansys2_node-1] [INFO] [1748541905.968717603] [planner]: [planner-popf] called with timeout 15.000000 seconds
[plansys2_node-1] [INFO] [1748541906.144270258] [executor]: Action calibrate timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748541906.147523658] [executor]: Action lock timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748541906.150948442] [executor]: Action move_robot timeout percentage -1.000000
[plansys2_node-1] getExpr: Error parsing expresion [(and (robot_at robot1 b1))]
[plansys2_node-1] getExpr: Error parsing expresion [(robot_at robot1 b1)]
[plansys2_node-1] [WARN] [1748541906.154251082] [rcl.logging_rosout]: Publisher already registered for node name: 'domain_expert_client'. If this is due to multiple nodes with the same name then all logs for the logger named 'domain_expert_client' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[plansys2_node-1] [WARN] [1748541906.188334775] [rcl.logging_rosout]: Publisher already registered for node name: 'problem_expert_client'. If this is due to multiple nodes with the same name then all logs for the logger named 'problem_expert_client' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[gantry_calibrate_action_node-3] [INFO] [1748541907.238350114] [gantry_calibrate_action_node]: GantryCalibrateActionNode::do_work() called
[gantry_lock_action_node-4] [INFO] [1748541907.238523961] [gantry_lock_action_node]: GantryLockActionNode::do_work() called
[gantry_lock_action_node-4] [INFO] [1748541907.238713494] [gantry_lock_action_node]: Sending request to gantry emulator service
[gantry_calibrate_action_node-3] [INFO] [1748541907.238571778] [gantry_calibrate_action_node]: Sending request to gantry emulator service
[plansys2_node-1] [ERROR] [1748541907.438445725] [executor]: [(move_robot gantry1 robot1 a1 b1):1001]Error checking at start reqs: (and (robot_at robot1 a1)(calibrated gantry1)(locked gantry1))
[robot_move_action_node-6] [INFO] [1748541908.539248511] [robot_move_action_node]: RobotMoveActionNode::do_work() called
[robot_move_action_node-6] [INFO] [1748541908.539514015] [robot_move_action_node]: Sending request to robot emulator service
[plansys2_node-1] [INFO] [1748541908.837913927] [executor]: Plan Succeeded
```