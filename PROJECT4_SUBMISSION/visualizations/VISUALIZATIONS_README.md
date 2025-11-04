# Visualizations

This directory contains all generated visualizations for the solution paths. Each visualization provides different perspectives on the motion planning solutions.

## Total Visualizations: 27 PNG files

### Pendulum Visualizations (18 files)

For each combination of planner and torque value, we have two plots:
- **Phase Space Plot**: Shows the trajectory in (θ, ω) phase space
- **Time Evolution Plot**: Shows θ and ω evolving over time

#### Torque = 3 (6 files)
- `pendulum_rrt_torque3_path_phase.png` - RRT phase space
- `pendulum_rrt_torque3_path_time.png` - RRT time evolution
- `pendulum_kpiece_torque3_path_phase.png` - KPIECE1 phase space
- `pendulum_kpiece_torque3_path_time.png` - KPIECE1 time evolution
- `pendulum_rgrrt_torque3_path_phase.png` - RG-RRT phase space
- `pendulum_rgrrt_torque3_path_time.png` - RG-RRT time evolution

#### Torque = 5 (6 files)
- `pendulum_rrt_torque5_path_phase.png` - RRT phase space
- `pendulum_rrt_torque5_path_time.png` - RRT time evolution
- `pendulum_kpiece_torque5_path_phase.png` - KPIECE1 phase space
- `pendulum_kpiece_torque5_path_time.png` - KPIECE1 time evolution
- `pendulum_rgrrt_torque5_path_phase.png` - RG-RRT phase space
- `pendulum_rgrrt_torque5_path_time.png` - RG-RRT time evolution

#### Torque = 10 (6 files)
- `pendulum_rrt_torque10_path_phase.png` - RRT phase space
- `pendulum_rrt_torque10_path_time.png` - RRT time evolution
- `pendulum_kpiece_torque10_path_phase.png` - KPIECE1 phase space
- `pendulum_kpiece_torque10_path_time.png` - KPIECE1 time evolution
- `pendulum_rgrrt_torque10_path_phase.png` - RG-RRT phase space
- `pendulum_rgrrt_torque10_path_time.png` - RG-RRT time evolution

### Car Visualizations (9 files)

For each planner, we have three plots:
- **Trajectory Plot**: 2D path showing x-y trajectory with orientation arrows
- **Workspace Plot**: Trajectory overlaid on obstacle environment
- **States Plot**: Evolution of all 4 states (x, y, θ, v) over time

#### RRT (3 files)
- `car_rrt_path_trajectory.png` - 2D trajectory with orientation
- `car_rrt_path_workspace.png` - Path through obstacle environment
- `car_rrt_path_states.png` - State evolution over time

#### KPIECE1 (3 files)
- `car_kpiece_path_trajectory.png` - 2D trajectory with orientation
- `car_kpiece_path_workspace.png` - Path through obstacle environment
- `car_kpiece_path_states.png` - State evolution over time

#### RG-RRT (3 files)
- `car_rgrrt_path_trajectory.png` - 2D trajectory with orientation
- `car_rgrrt_path_workspace.png` - Path through obstacle environment
- `car_rgrrt_path_states.png` - State evolution over time

## Key Observations from Visualizations

### Pendulum
1. **Higher torque = more direct paths**: Torque=10 shows almost straight trajectories to goal
2. **Phase space behavior**: All planners swing up and stabilize near (0, 0)
3. **RG-RRT efficiency**: Generally smoother, more direct paths in phase space
4. **KPIECE1 exploration**: Shows more systematic coverage of phase space

### Car
1. **Obstacle avoidance**: All planners successfully navigate around rectangular obstacles
2. **Non-holonomic constraints**: Clear turning radius constraints visible in trajectories
3. **RG-RRT path quality**: Shortest, smoothest paths with fewer waypoints
4. **KPIECE1 coverage**: More exploratory paths showing grid-based discretization
5. **State consistency**: All paths maintain valid (x, y, θ, v) states throughout

## Visualization Scripts

- `visualize_pendulum.py` - Generates phase space and time plots for pendulum
- `visualize_car.py` - Generates trajectory, workspace, and state plots for car

### Usage Example

```bash
python visualize_pendulum.py ../solution_paths/pendulum_rrt_torque3_path.txt
python visualize_car.py ../solution_paths/car_rgrrt_path.txt
```

## Technical Details

- **Format**: PNG with 300 DPI resolution
- **Dimensions**: 10×8 inches (pendulum), 15×5 inches (car)
- **Color scheme**: Blue for start, red for goal, distinct colors per plot
- **Generated with**: matplotlib, numpy

---
*All visualizations demonstrate successful motion planning with 100% success rate across all planners and configurations.*
