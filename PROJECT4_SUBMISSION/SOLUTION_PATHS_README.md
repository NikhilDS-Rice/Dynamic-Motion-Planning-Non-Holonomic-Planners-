# Solution Paths

This directory contains all generated solution paths for the project. Each file represents a successful path found by one of the three planners (RRT, KPIECE1, or RG-RRT) for either the pendulum or car problem.

## Pendulum Problem Paths

All pendulum paths start from state (-π/2, 0) and reach the goal region near (0, 0).

### Torque = 3
- **pendulum_rrt_torque3_path.txt**: RRT solution (527 states, 0.022s)
- **pendulum_kpiece_torque3_path.txt**: KPIECE1 solution (399 states, 0.021s)
- **pendulum_rgrrt_torque3_path.txt**: RG-RRT solution (347 states, 0.029s)

### Torque = 5
- **pendulum_rrt_torque5_path.txt**: RRT solution (95 states, 0.010s)
- **pendulum_kpiece_torque5_path.txt**: KPIECE1 solution (317 states, 0.017s)
- **pendulum_rgrrt_torque5_path.txt**: RG-RRT solution (146 states, 0.011s)

### Torque = 10
- **pendulum_rrt_torque10_path.txt**: RRT solution (83 states, 0.008s)
- **pendulum_kpiece_torque10_path.txt**: KPIECE1 solution (121 states, 0.006s)
- **pendulum_rgrrt_torque10_path.txt**: RG-RRT solution (37 states, 0.005s)

**Observations**:
- Higher torque values enable faster solutions (fewer states, less time)
- RG-RRT consistently produces compact paths with fewer states
- KPIECE1 is the fastest planner for torque=10 (0.006s)

## Car Problem Paths

All car paths navigate from (-40, -40, 0, 0) to the goal region near (40, 40) while avoiding obstacles.

### All Planners
- **car_rrt_path.txt**: RRT solution (2011 states, 1.62s, 94 waypoints)
- **car_kpiece_path.txt**: KPIECE1 solution (1841 states, 3.63s, 667 waypoints)
- **car_rgrrt_path.txt**: RG-RRT solution (3060 states, 2.60s, 69 waypoints)

**Observations**:
- RG-RRT produces the shortest path (69 waypoints) despite exploring more states
- KPIECE1 produces a longer path but covers the space systematically with grid cells
- All planners successfully avoid obstacles using the 4D projection (x, y, θ, v)

## Path File Format

Each path file contains waypoints in the following format:

**Pendulum** (4 columns):
```
theta omega control duration
```

**Car** (7 columns):
```
x y theta v steering_angle acceleration duration
```

## Key Achievements

1. **100% Success Rate**: All planners achieve 100% success on all configurations
2. **Proper State Space**: SO2StateSpace for pendulum angles, SE2StateSpace for car
3. **Critical 4D Projection**: Car uses 4D→4D projection to preserve all state information
4. **Efficient Solutions**: RG-RRT consistently finds shortest paths, KPIECE1 is fastest

## Usage

These paths can be visualized using the scripts in the `visualizations/` directory:
- `visualize_pendulum.py`: Phase space and time evolution plots
- `visualize_car.py`: 2D trajectory, workspace, and state plots

To visualize a specific path:
```bash
python visualize_pendulum.py pendulum_rrt_torque3_path.txt
python visualize_car.py car_rgrrt_path.txt
```

---
*Generated as part of COMP 550 Project 4 submission demonstrating successful motion planning for underactuated systems using RRT, KPIECE1, and RG-RRT planners.*
