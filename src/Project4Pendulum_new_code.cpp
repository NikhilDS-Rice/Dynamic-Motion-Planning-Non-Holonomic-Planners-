///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Shrikant & Nikhil
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
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/StateSpace.h>

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
        const auto *cs   = state->as<ompl::base::CompoundState>();
        const auto *th   = cs->as<ompl::base::SO2StateSpace::StateType>(0);
        const auto *wvec = cs->as<ompl::base::RealVectorStateSpace::StateType>(1);
    
        projection(0) = th->value;        // theta in [-pi, pi), wraps naturally
        projection(1) = wvec->values[0];  // omega
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
    namespace ob = ompl::base;
    namespace oc = ompl::control;

    // --- State space: SO2 (theta) × R^1 (omega) ---
    auto so2   = std::make_shared<ob::SO2StateSpace>();             // wraps angle
    auto wspace= std::make_shared<ob::RealVectorStateSpace>(1);     // omega
    ob::RealVectorBounds wBounds(1);
    wBounds.setLow(0, -10.0);
    wBounds.setHigh(0,  10.0);
    wspace->setBounds(wBounds);

    auto space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(so2,   1.0);
    space->addSubspace(wspace,1.0);
    space->lock();

    // --- Control space: 1D torque u ∈ [-torque, torque] ---
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 1);
    ob::RealVectorBounds uBounds(1);
    uBounds.setLow (0, -torque);
    uBounds.setHigh(0,  torque);
    cspace->setBounds(uBounds);

    // --- SimpleSetup + ODE propagator ---
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);
    auto si = ss->getSpaceInformation();
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(si, &pendulumODE);
    si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    // Tune control duration & integration step (important for KPIECE)
    si->setMinMaxControlDuration(1, 50);   // apply each control for 1..50 steps
    si->setPropagationStepSize(0.015);     // 15 ms per integration step

    // Pendulum has no obstacles: all states within bounds are valid
    ss->setStateValidityChecker([](const ob::State*){ return true; });

    // --- Start / Goal ---
    ob::ScopedState<> start(space), goal(space);
    start[0] = -M_PI/2.0;  // theta
    start[1] = 0.0;        // omega
    goal[0]  =  M_PI/2.0;  // theta
    goal[1]  =  0.0;       // omega

    // Loosen goal radius (was 0.05 and too strict for τ=3)
    ss->setStartAndGoalStates(start, goal, 0.35);

    // KPIECE needs a projection; keep your 2D (theta,omega) projection
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
        auto planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        // Set parameters for RG-RRT
        planner->setNumControlSamples(11);  // 11 evenly spaced torque values
        planner->setReachabilityDuration(0.05);  // Small time step for reachability
        ss->setPlanner(planner);
        plannerName = "rgrrt";
        std::cout << "Using RG-RRT planner" << std::endl;
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

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss, const std::string& logFilename = "pendulum_benchmark.log")
{
    // Create benchmark object
    ompl::tools::Benchmark b(*ss, "Pendulum Benchmark");
    
    // Add planners to benchmark
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    auto kpiece = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
    b.addPlanner(kpiece);
    
    // Add RG-RRT with parameters
    auto rgrrt = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
    rgrrt->setNumControlSamples(11);
    rgrrt->setReachabilityDuration(0.05);
    b.addPlanner(rgrrt);
    
    // Set benchmark parameters
    ompl::tools::Benchmark::Request req;
    req.maxTime = 30.0;
    req.maxMem = 2000.0;
    req.runCount = 50;
    req.displayProgress = true;
    
    // Run the benchmark
    b.benchmark(req);
    
    // Save results
    b.saveResultsToFile(logFilename.c_str());
    std::cout << "Benchmark results saved to " << logFilename << std::endl;
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
    {
        std::string logFilename = "pendulum_benchmark_torque" + std::to_string((int)torque) + ".log";
        benchmarkPendulum(ss, logFilename);
    }

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
