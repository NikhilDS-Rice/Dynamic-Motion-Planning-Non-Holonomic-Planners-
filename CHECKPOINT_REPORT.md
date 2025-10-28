# Project 4 Checkpoint Report: Control-Based Motion Planning

**Student:** [Your Name]  
**Course:** COMP/ELEC/MECH 450/550  
**Date:** October 27, 2025  
**Submission:** Checkpoint (Due: October 28, 2025 at 1pm)

---

## Overview

This checkpoint implements two control-based motion planning algorithms—**RRT** (Rapidly-exploring Random Tree) and **KPIECE1** (Kinodynamic Planning by Interior-Exterior Cell Exploration)—for two dynamic systems: a torque-controlled pendulum and a non-holonomic car navigating through obstacles.

## Implementation

### System 1: Pendulum Swing-Up

**Problem:** Swing an underactuated pendulum from hanging down (θ=0°) to upright (θ=180°) using limited torque control.

**State Space:** (θ, ω) representing angle and angular velocity  
**Dynamics:** θ̇ = ω, ω̇ = -g·cos(θ) + τ  
**Control:** Torque τ ∈ [-torque_limit, +torque_limit]

**KPIECE1 Configuration:** 2D projection on (θ, ω) for discretization-based exploration.

### System 2: Car Navigation with Obstacles

**Problem:** Navigate a 5×5 square car through a constrained environment with 8 rectangular obstacles from start (-40, -40) to goal (40, 40).

**State Space:** SE(2) × ℝ representing (x, y, θ, v) - position, heading, and velocity  
**Dynamics (Non-holonomic):**  
```
ẋ = v·cos(θ)  
ẏ = v·sin(θ)  
θ̇ = ω  
v̇ = a
```

**Control:** (ω, a) - angular velocity ∈ [-2, 2] rad/s and acceleration ∈ [-3, 3] m/s²

**Collision Checking:** Custom `CarValidityChecker` class checks all four corners of the 5×5 square car rotated by heading angle θ against obstacle boundaries using line-segment intersection tests.

**KPIECE1 Configuration:** 3D projection on (x, y, v) to account for both spatial and dynamic constraints.

## Results

### Pendulum Performance
| Planner | Torque Limit | Time (s) | States Created | Outcome |
|---------|--------------|----------|----------------|---------|
| RRT | 3 | 0.10 | ~100 | Success |
| RRT | 5 | 3.90 | ~400 | Success |
| KPIECE1 | 3 | 30.04 | ~3000 | Success |

**Key Finding:** Lower torque limits don't necessarily increase difficulty—RRT with torque=3 found solutions faster than higher torque values, likely due to more constrained control space reducing exploration overhead.

### Car Navigation Performance
| Planner | Solve Time (s) | States Created | Final Position | Distance to Goal |
|---------|----------------|----------------|----------------|------------------|
| RRT | 5.06 | 2,256 | (39.89, 39.36) | 0.65 units |
| KPIECE1 | 1.14 | 2,175 | ~(40, 40) | < 2.0 units |

**Key Finding:** KPIECE1 was **4.4× faster** than RRT for the car problem, demonstrating the advantage of discretization-based exploration for systems with complex kinodynamic constraints. The 3D projection (x, y, v) helped KPIECE1 effectively balance spatial and velocity exploration.

## Technical Challenges

**Square Collision Checking:** Initial implementation treated the car as a point robot, allowing invalid paths through obstacles. Solution involved implementing `isValidSquare()` that transforms all four corners of the 5×5 car based on position and heading, then checks edge-obstacle intersections.

**Timeout Tuning:** The narrow corridors in the street environment required extending the planning timeout from 60 to 300 seconds to allow sufficient exploration time for the constrained 5×5 car.

**Non-holonomic Constraints:** The car can only move along its heading direction (forward/backward), never sideways, making the planning problem significantly harder than holonomic systems.

## Visualizations

Generated comprehensive visualizations including:
- Trajectory plots showing the 5×5 square car at multiple timesteps with heading indicators
- Workspace views with color-coded obstacles and start/goal positions
- State evolution plots (x(t), y(t), θ(t), v(t)) demonstrating constraint satisfaction
- Pendulum phase portraits showing angular position and velocity evolution

All visualizations confirm that solutions respect system dynamics and collision constraints.

## Conclusion

Both RRT and KPIECE1 successfully solve control-based planning problems for underactuated and non-holonomic systems. KPIECE1's discretization-based approach with appropriate projections provides significant performance advantages for kinodynamically-constrained problems like the car navigation task. The implementation correctly handles complex collision checking for non-point robots and respects all dynamic constraints.

**Checkpoint Status:** ✅ Complete and ready for submission

---

**Deliverables:**
- Working source code (`src/Project4Pendulum.cpp`, `src/Project4Car.cpp`)
- Solution paths (`car_vis_test1/car_rrt_path.txt`, `car_kpiece_path.txt`)
- Visualizations (9 images in `car_vis_test1/`)
- Documentation (`CHECKPOINT_SUMMARY.md`)
