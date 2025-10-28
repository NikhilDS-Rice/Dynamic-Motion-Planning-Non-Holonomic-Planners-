# Project 4 - Visualization Results

## Generated Visualizations

### Pendulum System
1. **pendulum_path_phase.png** - Phase space plot (θ vs ω)
   - Shows the trajectory from hanging down (-π/2) to pointing up (π/2)
   - Green circle: Start state
   - Red star: Goal state
   - Blue arrows: Direction of motion

2. **pendulum_path_time.png** - States over time
   - Top plot: Angle (θ) evolution
   - Bottom plot: Angular velocity (ω) evolution

### Car System
1. **car_path_trajectory.png** - 2D workspace with obstacles
   - Shows the car navigating from (-40, -40) to (40, 40)
   - Gray rectangles: Obstacles (walls and buildings)
   - Green circle: Start position
   - Red star: Goal position
   - Red arrows: Car heading direction

2. **car_path_workspace.png** - Detailed workspace visualization
   - Color gradient: Time progression (dark to light)
   - Blue arrows: Car orientation at key points
   - Shows how the car maneuvers around obstacles

3. **car_path_states.png** - All states over time
   - X position, Y position, Heading (θ), and Velocity
   - Shows the complete state evolution

## How to Regenerate

```bash
# Pendulum visualizations
python visualize_pendulum.py pendulum_path.txt

# Car visualizations
python visualize_car.py car_path.txt

# Or generate all at once
python generate_all_visualizations.py
```

## Requirements
- Python 3.x
- numpy
- matplotlib

Install with: `pip install numpy matplotlib`

## Checkpoint Status ✅

✅ Pendulum planning with RRT and KPIECE1
✅ Car planning with RRT and KPIECE1  
✅ Solution paths generated
✅ Visualizations created
✅ Ready for 1-page report submission
