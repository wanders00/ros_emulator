## PlanSys2 Model

A PlanSys2 v2.0.18 model for the emulator. Developed for ROS2 Jazzy.

Inspiration taken from: 
- https://github.com/PlanSys2/ros2_planning_system_examples
- https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_simple_example
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

### Model output:
```bash
ros2 launch plansys2_model plansys2_model.launch.py
[INFO] [launch]: All log files can be found below /home/<user>/.ros/log/<log>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [plansys2_node-1]: process started with pid [68261]
[INFO] [gantry_move_action_node-2]: process started with pid [68262]
[INFO] [gantry_calibrate_action_node-3]: process started with pid [68263]
[INFO] [gantry_lock_action_node-4]: process started with pid [68264]
[INFO] [gantry_unlock_action_node-5]: process started with pid [68265]
[INFO] [robot_move_action_node-6]: process started with pid [68266]
[INFO] [robot_mount_action_node-7]: process started with pid [68267]
[INFO] [robot_unmount_action_node-8]: process started with pid [68268]
[INFO] [robot_check_mounted_action_node-9]: process started with pid [68269]
[gantry_lock_action_node-4] [INFO] [1748795524.182045502] [gantry_lock_action_node]: GantryLockActionNode constructed
[robot_mount_action_node-7] [INFO] [1748795524.182490412] [robot_mount_action_node]: RobotMountActionNode constructed
[robot_unmount_action_node-8] [INFO] [1748795524.183209889] [robot_unmount_action_node]: RobotUnmountActionNode constructed
[gantry_unlock_action_node-5] [INFO] [1748795524.186293638] [gantry_unlock_action_node]: GantryUnlockActionNode constructed
[robot_move_action_node-6] [INFO] [1748795524.192824323] [robot_move_action_node]: RobotMoveActionNode constructed
[gantry_move_action_node-2] [INFO] [1748795524.203437860] [gantry_move_action_node]: GantryMoveActionNode constructed
[gantry_calibrate_action_node-3] [INFO] [1748795524.205441087] [gantry_calibrate_action_node]: GantryCalibrateActionNode constructed
[robot_check_mounted_action_node-9] [INFO] [1748795524.208268984] [robot_check_mounted_action_node]: RobotCheckMountedActionNode constructed
[plansys2_node-1] [INFO] [1748795524.341706394] [domain_expert_lc_mngr]: Creating client for service [domain_expert/get_state]
[plansys2_node-1] [INFO] [1748795524.341793296] [domain_expert_lc_mngr]: Creating client for service [domain_expert/change_state]
[plansys2_node-1] [INFO] [1748795524.344358251] [executor_lc_mngr]: Creating client for service [executor/get_state]
[plansys2_node-1] [INFO] [1748795524.344400072] [executor_lc_mngr]: Creating client for service [executor/change_state]
[plansys2_node-1] [INFO] [1748795524.346746908] [planner_lc_mngr]: Creating client for service [planner/get_state]
[plansys2_node-1] [INFO] [1748795524.346796179] [planner_lc_mngr]: Creating client for service [planner/change_state]
[plansys2_node-1] [INFO] [1748795524.349411761] [problem_expert_lc_mngr]: Creating client for service [problem_expert/get_state]
[plansys2_node-1] [INFO] [1748795524.349460162] [problem_expert_lc_mngr]: Creating client for service [problem_expert/change_state]
[plansys2_node-1] [INFO] [1748795524.352219908] [planner]: [planner] Configuring...
[plansys2_node-1] [INFO] [1748795524.353251162] [planner]: Created solver : POPF of type plansys2/POPFPlanSolver
[plansys2_node-1] [INFO] [1748795524.353285863] [planner]: [planner] Solver Timeout 15
[plansys2_node-1] [INFO] [1748795524.355532036] [planner]: [planner] Configured
[plansys2_node-1] [INFO] [1748795524.355690180] [planner_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.355842463] [planner_lc_mngr]: Node planner_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748795524.355929121] [domain_expert]: [domain_expert] Configuring...
[plansys2_node-1] [INFO] [1748795524.358154424] [domain_expert]: Writing domain validation results to /tmp.
[plansys2_node-1] [INFO] [1748795524.544479317] [domain_expert]: [domain_expert] Configured
[plansys2_node-1] [INFO] [1748795524.544677177] [domain_expert_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.544880631] [domain_expert_lc_mngr]: Node domain_expert_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748795524.544965958] [problem_expert]: [problem_expert] Configuring...
[plansys2_node-1] [INFO] [1748795524.545169753] [problem_expert]: [problem_expert] Configured
[plansys2_node-1] [INFO] [1748795524.545249224] [problem_expert_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.545385278] [problem_expert_lc_mngr]: Node problem_expert_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748795524.545485686] [executor]: [executor] Configuring...
[plansys2_node-1] [INFO] [1748795524.636498414] [planner_client]: Planner CLient created with timeout 15
[plansys2_node-1] [INFO] [1748795524.636564706] [executor]: [executor] Configured
[plansys2_node-1] [INFO] [1748795524.636832346] [executor_lc_mngr]: Transition 1 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.637000430] [executor_lc_mngr]: Node executor_lc_mngr has current state inactive.
[plansys2_node-1] [INFO] [1748795524.637091943] [domain_expert]: [domain_expert] Activating...
[plansys2_node-1] [INFO] [1748795524.637102053] [domain_expert]: [domain_expert] Activated
[plansys2_node-1] [INFO] [1748795524.637151399] [domain_expert_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.637240036] [problem_expert]: [problem_expert] Activating...
[plansys2_node-1] [INFO] [1748795524.637252267] [problem_expert]: [problem_expert] Activated
[plansys2_node-1] [INFO] [1748795524.637297623] [problem_expert_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.637364299] [planner]: [planner] Activating...
[plansys2_node-1] [INFO] [1748795524.637372949] [planner]: [planner] Activated
[plansys2_node-1] [INFO] [1748795524.637425700] [planner_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.637516088] [executor]: [executor] Activating...
[plansys2_node-1] [INFO] [1748795524.637527868] [executor]: [executor] Activated
[plansys2_node-1] [INFO] [1748795524.637575479] [executor_lc_mngr]: Transition 3 successfully triggered.
[plansys2_node-1] [INFO] [1748795524.637703967] [domain_expert_lc_mngr]: Node domain_expert_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748795524.637829870] [problem_expert_lc_mngr]: Node problem_expert_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748795524.637941022] [planner_lc_mngr]: Node planner_lc_mngr has current state active.
[plansys2_node-1] [INFO] [1748795524.638069595] [executor_lc_mngr]: Node executor_lc_mngr has current state active.
[plansys2_node-1] Skipping adding constant to problem :object: none tool
[plansys2_node-1] Skipping adding constant to problem :object: unknown tool
[plansys2_node-1] [INFO] [1748795530.599339202] [planner]: Writing planning results to /tmp.
[plansys2_node-1] [INFO] [1748795530.599629859] [planner]: [planner-popf] called with timeout 15.000000 seconds
[plansys2_node-1] Skipping adding constant to problem :object: none tool
[plansys2_node-1] Skipping adding constant to problem :object: unknown tool
[plansys2_node-1] [INFO] [1748795532.930437893] [planner]: Writing planning results to /tmp.
[plansys2_node-1] [INFO] [1748795532.930624438] [planner]: [planner-popf] called with timeout 15.000000 seconds
[plansys2_node-1] [INFO] [1748795533.104691759] [executor]: Action check_mounted timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748795533.107895085] [executor]: Action unlock timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748795533.110913247] [executor]: Action calibrate timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748795533.114448386] [executor]: Action lock timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748795533.118122502] [executor]: Action move_robot timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748795533.121609170] [executor]: Action move_robot timeout percentage -1.000000
[plansys2_node-1] [INFO] [1748795533.125449436] [executor]: Action mount timeout percentage -1.000000
[plansys2_node-1] getExpr: Error parsing expresion [(and (mounted robot1))]
[plansys2_node-1] getExpr: Error parsing expresion [(mounted robot1)]
[plansys2_node-1] getExpr: Error parsing expresion [(and (tool_taken suction_tool))]
[plansys2_node-1] getExpr: Error parsing expresion [(tool_taken suction_tool)]
[plansys2_node-1] [WARN] [1748795533.130199423] [rcl.logging_rosout]: Publisher already registered for node name: 'domain_expert_client'. If this is due to multiple nodes with the same name then all logs for the logger named 'domain_expert_client' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[plansys2_node-1] [WARN] [1748795533.169615307] [rcl.logging_rosout]: Publisher already registered for node name: 'problem_expert_client'. If this is due to multiple nodes with the same name then all logs for the logger named 'problem_expert_client' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[robot_check_mounted_action_node-9] [INFO] [1748795534.223227092] [robot_check_mounted_action_node]: RobotCheckMountedActionNode::do_work() called
[robot_check_mounted_action_node-9] [INFO] [1748795534.223408417] [robot_check_mounted_action_node]: Sending request to robot emulator service
[gantry_unlock_action_node-5] [INFO] [1748795534.223433877] [gantry_unlock_action_node]: GantryUnlockActionNode::do_work() called
[gantry_unlock_action_node-5] [INFO] [1748795534.223626322] [gantry_unlock_action_node]: Sending request to gantry emulator service
[gantry_calibrate_action_node-3] [INFO] [1748795535.423707469] [gantry_calibrate_action_node]: GantryCalibrateActionNode::do_work() called
[gantry_calibrate_action_node-3] [INFO] [1748795535.423918904] [gantry_calibrate_action_node]: Sending request to gantry emulator service
[gantry_lock_action_node-4] [INFO] [1748795536.623626195] [gantry_lock_action_node]: GantryLockActionNode::do_work() called
[gantry_lock_action_node-4] [INFO] [1748795536.623821500] [gantry_lock_action_node]: Sending request to gantry emulator service
[robot_move_action_node-6] [INFO] [1748795537.823785310] [robot_move_action_node]: RobotMoveActionNode::do_work() called
[robot_move_action_node-6] [INFO] [1748795537.823988425] [robot_move_action_node]: Sending request to robot emulator service
[robot_move_action_node-6] [INFO] [1748795539.023781036] [robot_move_action_node]: RobotMoveActionNode::do_work() called
[robot_move_action_node-6] [INFO] [1748795539.023936009] [robot_move_action_node]: Sending request to robot emulator service
[plansys2_node-1] [ERROR] [1748795539.222650499] [executor]: [(mount robot1 suction_tool gantry1 suction_tool_rack):5002]Error checking over all reqs: (and (robot_at robot1 suction_tool_rack)(tool_at suction_tool suction_tool_rack)(locked gantry1))
[robot_mount_action_node-7] [INFO] [1748795540.323755377] [robot_mount_action_node]: RobotMountActionNode::do_work() called
[robot_mount_action_node-7] [INFO] [1748795540.323975342] [robot_mount_action_node]: Sending request to robot emulator service
[plansys2_node-1] [INFO] [1748795540.622250758] [executor]: Plan Succeeded
```

### PlanSys2 Terminal output:
```bash
ros2 run plansys2_terminal plansys2_terminal
[INFO] [1748795525.867359849] [planner_client]: Planner CLient created with timeout 0
[INFO] [1748795526.187833257] [terminal]: No problem file specified.
ROS2 Planning System console. Type "quit" to finish
> set instance gantry1 gantry

set instance robot1 robot

set instance suction_tool tool
set instance gripper_tool tool
set instance none tool
set instance unknown tool

set instance start location
set instance suction_tool_rack location
set instance gripper_tool_rack location

set predicate (attached robot1 unknown)

set predicate (robot_at robot1 start)

set predicate (unmounted robot1)

set predicate (gantry_at gantry1 start)

set predicate (uncalibrated gantry1)
set predicate (locked gantry1)

get plan (and(mounted robot1)(tool_taken suction_tool))
getExpr: Error parsing expresion [ (and(mounted robot1)(tool_taken suction_tool))]
getExpr: Error parsing expresion [(mounted robot1)]
getExpr: Error parsing expresion [(tool_taken suction_tool)]
[ERROR] [1748795530.597683924] [planner_client]: Get Plan service called with negative timed out:0. Setting to 15 seconds
[INFO] [1748795530.597757426] [planner_client]: Get Plan service call with time out 15
plan: 
0:      (check_mounted robot1)  [1]
0:      (unlock gantry1)        [1]
1.001:  (calibrate gantry1)     [1]
2.001:  (lock gantry1)  [1]
3.001:  (move_robot gantry1 robot1 start gripper_tool_rack)     [1]
4.002:  (move_robot gantry1 robot1 gripper_tool_rack suction_tool_rack) [1]
5.002:  (mount robot1 suction_tool gantry1 suction_tool_rack)   [1]
> run
[ERROR] [1748795532.929131162] [planner_client]: Get Plan service called with negative timed out:0. Setting to 15 seconds
[INFO] [1748795532.929195973] [planner_client]: Get Plan service call with time out 15
[INFO] [1748795542.887119556] [executor_client]: Plan Succeeded

Successful finished
```