///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Benchmarking
#include <ompl/tools/benchmark/Benchmark.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // Project the 2D state space (theta, omega) to 2D
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Project onto (theta, omega) - which is already our state space
        const auto *rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection(0) = rvstate->values[0]; // theta
        projection(1) = rvstate->values[1]; // omega
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // Retrieve the control input (torque)
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];
    
    // Extract current state: q[0] = theta, q[1] = omega
    const double theta = q[0];
    const double omega = q[1];
    
    // Gravity constant
    const double g = 9.81;
    
    // Pendulum dynamics: q_dot = [omega, -g*cos(theta) + torque]
    qdot.resize(q.size(), 0);
    qdot[0] = omega;                      // theta_dot = omega
    qdot[1] = -g * cos(theta) + torque;   // omega_dot = -g*cos(theta) + torque
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // Create the state space: 2D (theta, omega)
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    
    // Set bounds for the state space
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);      // theta lower bound
    bounds.setHigh(0, M_PI);      // theta upper bound
    bounds.setLow(1, -10);        // omega lower bound (angular velocity)
    bounds.setHigh(1, 10);        // omega upper bound
    space->setBounds(bounds);
    
    // Create the control space: 1D (torque)
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 1);
    
    // Set bounds for the control space
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);
    
    // Create SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);
    
    // Set the ODE solver
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    
    // Set state validity checker
    // For pendulum, all states within the state space bounds are valid (no obstacles)
    // The bounds checking is already done by the state space
    ss->setStateValidityChecker([](const ompl::base::State * /*state*/) {
        return true;
    });
    
    // Set the start and goal states
    ompl::base::ScopedState<> start(space);
    start[0] = -M_PI / 2.0;  // theta = -pi/2 (hanging down)
    start[1] = 0.0;           // omega = 0 (at rest)
    
    ompl::base::ScopedState<> goal(space);
    goal[0] = M_PI / 2.0;    // theta = pi/2 (pointing up)
    goal[1] = 0.0;            // omega = 0 (at rest)
    
    ss->setStartAndGoalStates(start, goal, 0.05);  // 0.05 is the goal radius tolerance
    
    // Register the projection
    space->registerDefaultProjection(std::make_shared<PendulumProjection>(space.get()));
    
    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    std::string plannerName;
    
    // Set the planner based on user choice
    if (choice == 1)  // RRT
    {
        auto planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        plannerName = "rrt";
        std::cout << "Using RRT planner" << std::endl;
    }
    else if (choice == 2)  // KPIECE1
    {
        auto planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        plannerName = "kpiece";
        std::cout << "Using KPIECE1 planner" << std::endl;
    }
    else if (choice == 3)  // RG-RRT
    {
        // TODO: Implement RG-RRT later
        plannerName = "rgrrt";
        std::cout << "RG-RRT not yet implemented" << std::endl;
        return;
    }
    
    // Attempt to solve the problem within 30 seconds
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(30.0);
    
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        
        // Print the path to screen
        ss->getSolutionPath().printAsMatrix(std::cout);
        
        // Save the solution path to a file with unique name
        std::string filename = "output/pendulum_" + plannerName + "_path.txt";
        std::ofstream fout(filename);
        ss->getSolutionPath().printAsMatrix(fout);
        fout.close();
        
        std::cout << "Solution saved to " << filename << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // Create benchmark object
    ompl::tools::Benchmark b(*ss, "Pendulum Benchmark");
    
    // Add planners to benchmark
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    // TODO: Add RG-RRT when implemented
    // b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    
    // Set benchmark parameters
    ompl::tools::Benchmark::Request req;
    req.maxTime = 30.0;
    req.maxMem = 1000.0;
    req.runCount = 20;
    req.displayProgress = true;
    
    // Run the benchmark
    b.benchmark(req);
    
    // Save results
    b.saveResultsToFile("pendulum_benchmark.log");
    std::cout << "Benchmark results saved to pendulum_benchmark.log" << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
