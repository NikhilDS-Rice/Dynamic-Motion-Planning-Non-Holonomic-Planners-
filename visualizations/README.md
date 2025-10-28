# Project 4 - Checkpoint Visualizations

## Generated Visualizations

### Pendulum System

#### RRT Planner (Torque = 10)
- **pendulum_rrt_path_phase.png** - Phase space trajectory (θ vs ω)
  - Shows efficient swing-up motion
  - 15 waypoints
  - Solution time: 3.88 seconds
  
- **pendulum_rrt_path_time.png** - State evolution over time
  - Angle and angular velocity progression

#### KPIECE1 Planner (Torque = 3)
- **pendulum_kpiece_path_phase.png** - Phase space trajectory
  - More complex path with systematic exploration
  - 85 waypoints (much longer path)
  - Solution time: 30.04 seconds
  - 408,846 states explored
  
- **pendulum_kpiece_path_time.png** - State evolution over time
  - Shows gradual energy buildup

### Car System

#### RRT Planner
- **car_rrt_path_trajectory.png** - 2D workspace view
  - Direct path with obstacle avoidance
  - 19 waypoints
  - Solution time: 0.16 seconds
  
- **car_rrt_path_workspace.png** - Detailed workspace with car orientations
  - Color gradient shows time progression
  - Blue arrows show car heading
  
- **car_rrt_path_states.png** - All states (x, y, θ, v) over time
  - Shows smooth velocity changes
  - Non-holonomic constraints respected

#### KPIECE1 Planner
- **car_kpiece_path_trajectory.png** - 2D workspace view
  - More exploration, longer path
  - 57 waypoints
  - Solution time: 0.04 seconds (faster!)
  
- **car_kpiece_path_workspace.png** - Detailed workspace
  - Shows systematic exploration pattern
  
- **car_kpiece_path_states.png** - All states over time
  - More complex state evolution

## Summary Statistics

### Pendulum
| Planner | Time (s) | States | Waypoints | File Size |
|---------|----------|--------|-----------|-----------|
| RRT     | 3.88     | 11,641 | 15        | 144 KB (phase) |
| KPIECE1 | 30.04    | 408,846 | 85        | 144 KB (phase) |

### Car
| Planner | Time (s) | States | Waypoints | Trajectory Size |
|---------|----------|--------|-----------|-----------------|
| RRT     | 0.16     | 87     | 19        | 85 KB           |
| KPIECE1 | 0.04     | 96     | 57        | 109 KB          |

## Key Observations

1. **RRT vs KPIECE1 Trade-offs:**
   - RRT: Faster for pendulum, shorter paths
   - KPIECE1: More systematic, sometimes faster (car), longer paths

2. **Pendulum Torque Effects:**
   - Lower torque (3) requires more complex trajectories
   - KPIECE1 with torque=3 takes 30s but finds solution

3. **Car Planning:**
   - Both planners very efficient (<0.2s)
   - Simple environment allows quick solutions
   - Non-holonomic constraints clearly visible in trajectories

4. **Path Quality:**
   - RRT: More direct but may not be optimal
   - KPIECE1: More waypoints, more thorough exploration

## Files for Checkpoint Report

**Best images for 1-page report:**
1. `pendulum_kpiece_path_phase.png` - Shows complex dynamics
2. `car_rrt_path_workspace.png` - Shows obstacle avoidance
3. `car_kpiece_path_trajectory.png` - Shows planning success

**Total:** 10 high-quality visualization images (1.28 MB)

## Checkpoint Status: ✅ COMPLETE

- ✅ RRT planner working for both systems
- ✅ KPIECE1 planner working for both systems
- ✅ Custom projections implemented
- ✅ Multiple test runs completed
- ✅ Comprehensive visualizations generated
- ✅ Ready for report submission
