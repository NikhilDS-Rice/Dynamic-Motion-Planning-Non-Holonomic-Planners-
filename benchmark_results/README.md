# Benchmark Results - Project 4
## Date: November 2-3, 2025

## Folder Structure

```
benchmark_results/
├── README.md                          (this file)
├── pendulum_benchmark.log             (Benchmark results for pendulum problem)
└── output/                            (Path outputs from individual test runs)
    ├── RESULTS_SUMMARY.txt            (Previous checkpoint results)
    ├── pendulum_rrt_path.txt          (Pendulum - RRT path)
    ├── pendulum_kpiece_path.txt       (Pendulum - KPIECE1 path)
    ├── pendulum_rgrrt_path.txt        (Pendulum - RG-RRT path)
    ├── car_rrt_path.txt               (Car - RRT path)
    ├── car_kpiece_path.txt            (Car - KPIECE1 path)
    └── car_rgrrt_path.txt             (Car - RG-RRT path)
```

## Benchmark Configuration

### Pendulum Problem
- **Planners**: RRT, KPIECE1, RG-RRT
- **Runs per planner**: 20
- **Timeout**: 30 seconds
- **Torque setting**: 10
- **Log file**: `pendulum_benchmark.log`

### Car Problem
- **Planners**: RRT, KPIECE1, RG-RRT
- **Runs per planner**: 20
- **Timeout**: 300 seconds (5 minutes)
- **Log file**: `car_benchmark.log` (not yet run)

## RG-RRT Parameters
- **numControlSamples**: 11
- **reachabilityDuration**: 0.05
- **goalBias**: 0.05

## Individual Test Results (from output/)

### Pendulum Solutions
- **RRT**: 496 bytes path file
- **KPIECE1**: 2,859 bytes path file
- **RG-RRT**: 324 bytes path file (torque=10, 0.863s solve time)

### Car Solutions
- **RRT**: 4,292 bytes path file
- **KPIECE1**: 5,826 bytes path file
- **RG-RRT**: 5,103 bytes path file (4.80s solve time, 3,929 states)

## Analysis Notes

The pendulum benchmark has been completed (60 runs total). The car benchmark is pending.

To analyze the benchmark results:
1. Parse `pendulum_benchmark.log` for statistics
2. Extract: solve time, success rate, path length, number of states
3. Compare RG-RRT performance against RRT and KPIECE1
4. Generate visualizations (box plots, bar charts)

## Files Copied from Docker
All files were copied from Docker container `comp450-workspace:/root/project4/` on November 3, 2025 at 03:50 UTC.

## Next Steps
1. ✅ Pendulum benchmark complete
2. 🔲 Run car benchmark (estimated 5 hours)
3. 🔲 Statistical analysis of results
4. 🔲 Create comparison visualizations
5. 🔲 Write final report
