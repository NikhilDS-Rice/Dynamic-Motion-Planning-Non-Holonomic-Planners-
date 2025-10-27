///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Benchmarking
#include <ompl/tools/benchmark/Benchmark.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // Project to 3D: (x, y, velocity)
        // Including velocity helps KPIECE understand the dynamic constraints
        return 3;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Extract the components from the compound state space
        const auto *compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto *se2state = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto *velState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);
        
        projection(0) = se2state->getX();      // x position
        projection(1) = se2state->getY();      // y position
        projection(2) = velState->values[0];   // velocity
    }
};

void carODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // Retrieve the control inputs
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double omega = u[0];      // angular velocity
    const double acceleration = u[1]; // forward acceleration
    
    // Extract current state: q[0] = x, q[1] = y, q[2] = theta, q[3] = v
    const double theta = q[2];
    const double v = q[3];
    
    // Car dynamics:
    // x_dot = v * cos(theta)
    // y_dot = v * sin(theta)
    // theta_dot = omega
    // v_dot = acceleration
    qdot.resize(q.size(), 0);
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = omega;
    qdot[3] = acceleration;
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // Create a simple street environment with obstacles
    // The environment is roughly 100x100 units
    
    // Left wall
    Rectangle r1;
    r1.x = -50; r1.y = -50;
    r1.width = 5; r1.height = 100;
    obstacles.push_back(r1);
    
    // Right wall
    Rectangle r2;
    r2.x = 45; r2.y = -50;
    r2.width = 5; r2.height = 100;
    obstacles.push_back(r2);
    
    // Top wall
    Rectangle r3;
    r3.x = -50; r3.y = 45;
    r3.width = 100; r3.height = 5;
    obstacles.push_back(r3);
    
    // Bottom wall
    Rectangle r4;
    r4.x = -50; r4.y = -50;
    r4.width = 100; r4.height = 5;
    obstacles.push_back(r4);
    
    // Middle obstacle (like a building or parked car)
    Rectangle r5;
    r5.x = -10; r5.y = -10;
    r5.width = 20; r5.height = 20;
    obstacles.push_back(r5);
    
    // Additional obstacle
    Rectangle r6;
    r6.x = 15; r6.y = -30;
    r6.width = 15; r6.height = 10;
    obstacles.push_back(r6);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // Create a compound state space: SE(2) for (x, y, theta) and R for velocity
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();
    
    // Add SE2 component (x, y, theta)
    auto se2 = std::make_shared<ompl::base::SE2StateSpace>();
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-50);
    bounds.setHigh(50);
    se2->setBounds(bounds);
    space->addSubspace(se2, 1.0);
    
    // Add velocity component
    auto velSpace = std::make_shared<ompl::base::RealVectorStateSpace>(1);
    ompl::base::RealVectorBounds velBounds(1);
    velBounds.setLow(-10);  // Can go in reverse
    velBounds.setHigh(10);  // Forward velocity limit
    velSpace->setBounds(velBounds);
    space->addSubspace(velSpace, 0.5);
    
    // Create control space: (omega, acceleration)
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);
    
    // Set control bounds
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -2.0);   // omega lower bound
    cbounds.setHigh(0, 2.0);   // omega upper bound
    cbounds.setLow(1, -3.0);   // acceleration lower bound
    cbounds.setHigh(1, 3.0);   // acceleration upper bound
    cspace->setBounds(cbounds);
    
    // Create SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);
    
    // Set the ODE solver
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    
    // Set state validity checker
    ss->setStateValidityChecker([&obstacles, &velBounds](const ompl::base::State *state) {
        // Extract the SE2 component for collision checking
        const auto *compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto *se2state = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto *velState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);
        
        // Check velocity bounds
        double v = velState->values[0];
        if (v < velBounds.low[0] || v > velBounds.high[0])
            return false;
        
        // Check collision - treating car as a point
        return isValidStatePoint(se2state, obstacles);
    });
    
    // Set the start state
    ompl::base::ScopedState<> start(space);
    start[0] = -40.0;  // x
    start[1] = -40.0;  // y
    start[2] = 0.0;    // theta
    start[3] = 0.0;    // velocity
    
    // Set the goal state
    ompl::base::ScopedState<> goal(space);
    goal[0] = 40.0;    // x
    goal[1] = 40.0;    // y
    goal[2] = 0.0;     // theta
    goal[3] = 0.0;     // velocity
    
    ss->setStartAndGoalStates(start, goal, 2.0);  // Goal tolerance
    
    // Register the projection
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));
    
    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // Set the planner based on user choice
    if (choice == 1)  // RRT
    {
        auto planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        std::cout << "Using RRT planner" << std::endl;
    }
    else if (choice == 2)  // KPIECE1
    {
        auto planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        std::cout << "Using KPIECE1 planner" << std::endl;
    }
    else if (choice == 3)  // RG-RRT
    {
        // TODO: Implement RG-RRT later
        std::cout << "RG-RRT not yet implemented" << std::endl;
        return;
    }
    
    // Attempt to solve the problem within 60 seconds
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(60.0);
    
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        
        // Print the path to screen
        ss->getSolutionPath().printAsMatrix(std::cout);
        
        // Save the solution path to a file
        std::ofstream fout("car_path.txt");
        ss->getSolutionPath().printAsMatrix(fout);
        fout.close();
        
        std::cout << "Solution saved to car_path.txt" << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // Create benchmark object
    ompl::tools::Benchmark b(*ss, "Car Benchmark");
    
    // Add planners to benchmark
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    // TODO: Add RG-RRT when implemented
    // b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    
    // Set benchmark parameters
    ompl::tools::Benchmark::Request req;
    req.maxTime = 60.0;
    req.maxMem = 1000.0;
    req.runCount = 20;
    req.displayProgress = true;
    
    // Run the benchmark
    b.benchmark(req);
    
    // Save results
    b.saveResultsToFile("car_benchmark.log");
    std::cout << "Benchmark results saved to car_benchmark.log" << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
