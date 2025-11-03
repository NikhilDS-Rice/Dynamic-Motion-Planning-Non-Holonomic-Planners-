# RG-RRT Implementation Summary
## Date: November 2, 2025

## Overview
Successfully implemented RG-RRT (Reachability-Guided RRT) planner based on Shkolnik, Walter, and Tedrake (ICRA 2009) for COMP 450/550 Project 4.

## Implementation Details

### Files Created/Modified

1. **RG-RRT.h** (218 lines)
   - Complete header defining RGRRT class extending ompl::base::Planner
   - Motion class with `std::vector<base::State*> reachableSet` member
   - Parameters: numControlSamples (11), reachabilityDuration (0.05), goalBias (0.05)
   - Methods: computeReachableSet(), isReachableFromMotion(), solve()

2. **RG-RRT.cpp** (414 lines)
   - Full implementation of RG-RRT algorithm
   - `computeReachableSet()`: Samples 11 evenly-spaced control values, propagates for 0.05s
   - Handles both 1D control (pendulum torque) and 2D control (car)
   - `isReachableFromMotion()`: Implements rejection criterion from paper
   - `solve()`: Main planning loop with sample rejection logic
   - Tracks rejected samples for analysis

3. **Project4Pendulum.cpp**
   - Added RG-RRT option (choice 3) in planPendulum()
   - Updated benchmarkPendulum() to include RG-RRT with parameters
   - Benchmark: 20 runs, 30s timeout

4. **Project4Car.cpp**
   - Added RG-RRT option (choice 3) in planCar()
   - Implemented benchmarkCar() from scratch with RG-RRT
   - Increased timeout from 5s to 300s for complex car problem
   - Benchmark: 20 runs, 300s timeout

## Algorithm Implementation

### Rejection Criterion (Corrected)
**Paper Definition**: Reject qrand if d(qnear, qrand) < d(r, qrand) for ALL r ∈ R(qnear)

**Implementation Logic**:
```cpp
- If reachable set is empty → ACCEPT (no rejection info)
- Check if qnear is closer than ALL reachable states
- If qnear IS closest → REJECT (qrand in "shadow" region)
- If ANY reachable state is closer → ACCEPT (worth exploring)
```

### Key Bug Fix
**Initial bug**: Logic was inverted, rejecting ALL samples (35M+ samples rejected!)
**Fix**: Corrected the rejection criterion to properly accept samples outside reachable regions

## Test Results

### Pendulum (Torque = 10)
- ✅ Solution found: 0.86 seconds
- States created: 6,655
- Samples rejected: 0 (all accepted)
- Path: 10 waypoints

### Car Problem
- ✅ Solution found: 4.80 seconds
- States created: 3,929
- Total samples: 6,367 (rejection working)
- Path: 85 waypoints

## Benchmarking Setup

### Pendulum Benchmark
- Planners: RRT, KPIECE1, RG-RRT
- Runs: 20 per planner
- Timeout: 30 seconds
- Output: `pendulum_benchmark.log`

### Car Benchmark
- Planners: RRT, KPIECE1, RG-RRT
- Runs: 20 per planner
- Timeout: 300 seconds (5 minutes)
- Output: `car_benchmark.log`

## Parameters Used

### RG-RRT Parameters
- **numControlSamples**: 11 (evenly spaced across control bounds)
- **reachabilityDuration**: 0.05 seconds (small time step)
- **goalBias**: 0.05 (5% goal biasing)

### Rationale
- 11 samples provide good coverage without excessive computation
- 0.05s duration balances exploration vs computation cost
- Parameters same for both pendulum and car for fair comparison

## Next Steps (Exercise 4)

1. ✅ **Benchmarking setup complete**
2. ⏳ **Running pendulum benchmark** (in progress)
3. 🔲 **Run car benchmark** (after pendulum completes)
4. 🔲 **Statistical analysis**:
   - Mean solve time
   - Standard deviation
   - Success rate
   - Path length
   - Number of states
5. 🔲 **Final report**:
   - Quantitative comparison
   - RG-RRT vs RRT trade-offs
   - Performance analysis
   - Visualizations

## Key Insights

### RG-RRT Advantages
- Reduces wasted expansions in already-explored regions
- Reachable set provides local reachability information
- Can be more efficient than standard RRT in cluttered spaces

### RG-RRT Overhead
- Computing reachable sets adds computational cost
- 11 control samples × propagation per new node
- Trade-off: fewer tree nodes vs per-node computation

### Implementation Challenges
1. Correct interpretation of rejection criterion from paper
2. Handling both 1D and 2D control spaces
3. Memory management for reachable set states
4. Balancing parameters (samples, duration) for performance

## Files in Docker
```
/root/project4/
├── src/
│   ├── RG-RRT.h
│   ├── RG-RRT.cpp
│   ├── Project4Pendulum.cpp
│   ├── Project4Car.cpp
│   ├── CollisionChecking.cpp
│   └── CollisionChecking.h
├── output/
│   ├── pendulum_rgrrt_path.txt
│   └── car_rgrrt_path.txt
├── pendulum_benchmark.log (in progress)
├── Project4Pendulum (executable)
└── Project4Car (executable)
```

## Compilation
```bash
cd /root/project4
make clean
make
```

## Running Benchmarks
```bash
# Pendulum
./Project4Pendulum
# Enter: 2 (Benchmark), 1 (Torque=3)

# Car
./Project4Car  
# Enter: 2 (Benchmark)
```

## Status
- ✅ Implementation: Complete
- ✅ Testing: Successful on both problems
- ⏳ Benchmarking: In progress (pendulum)
- 🔲 Analysis: Pending benchmark completion
- 🔲 Final Report: Due Nov 4, 1pm
