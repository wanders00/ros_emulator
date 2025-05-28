## PlanSys2 Model

A PlanSys2 v2.0.18 model for the emulator. Developed for ROS2 Jazzy.

Inspiration taken from: 
- https://github.com/PlanSys2/ros2_planning_system_examples
- https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_bt_example
- https://plansys2.github.io/

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

## Example run:

```bash
ros2 launch plansys2_model plansys2_model.launch.py
<todo: add example run output>
```