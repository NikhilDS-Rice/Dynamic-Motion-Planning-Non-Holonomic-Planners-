///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Shrikant & Nikhil
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>  

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

constexpr double CAR_SIZE = 1.5; 

// Custom Goal Region that uses TRUE Euclidean distance (not normalized)
class CarGoalRegion : public ob::GoalRegion
{
public:
    CarGoalRegion(const ob::SpaceInformationPtr &si, double x, double y, double theta, double v, double tolerance)
        : ob::GoalRegion(si), goalX_(x), goalY_(y), goalTheta_(theta), goalV_(v)
    {
        setThreshold(tolerance);
    }

    double distanceGoal(const ob::State *state) const override
    {
        const auto *cs = state->as<ob::CompoundState>();
        const auto *se2 = cs->as<ob::SE2StateSpace::StateType>(0);
        const auto *vvec = cs->as<ob::RealVectorStateSpace::StateType>(1);
        
        double x = se2->getX();
        double y = se2->getY();
        double theta = se2->getYaw();
        double v = vvec->values[0];
        
        // TRUE EUCLIDEAN DISTANCE in physical space (not normalized!)
        double dx = x - goalX_;
        double dy = y - goalY_;
        double spatialDist = std::sqrt(dx*dx + dy*dy);
        
        // For orientation, compute shortest angular distance
        double dtheta = std::abs(theta - goalTheta_);
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        dtheta = std::abs(dtheta);
        
        // Return spatial distance as primary metric
        // (we care most about reaching the position)
        return spatialDist;
    }

private:
    double goalX_, goalY_, goalTheta_, goalV_;
}; 

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // Project the 4D state space to 4D to keep ALL information
        return 4;
    }

    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd> projection ) const override
    {
        const auto *cs = state->as<ob::CompoundState>();
        const auto *se2 = cs->as<ob::SE2StateSpace::StateType>(0);
        const auto *vvec = cs->as<ob::RealVectorStateSpace::StateType>(1);
        
        projection[0] = se2->getX();
        projection[1] = se2->getY();
        projection[2] = se2->getYaw();    // Keep orientation!
        projection[3] = vvec->values[0];  // Keep velocity!
    }
};

void carODE(const ompl::control::ODESolver::StateType &  q , const ompl::control::Control * control ,
            ompl::control::ODESolver::StateType & qdot )
{
    const auto *u = control->as<oc::RealVectorControlSpace::ControlType>();
    const double x     = q[0];
    const double y     = q[1];
    const double theta = q[2];
    const double v     = q[3];

    const double omega = u->values[0];
    const double vdot  = u->values[1];

    (void)x; (void)y;

    qdot.resize(4);
    qdot[0] = v * std::cos(theta); 
    qdot[1] = v * std::sin(theta); 
    qdot[2] = omega;              
    qdot[3] = vdot;                
}

class CarValidityChecker : public ob::StateValidityChecker {
    public:
        CarValidityChecker(const ob::SpaceInformationPtr& si,
                           const std::vector<Rectangle>& obstacles,
                           double carSide)
            : ob::StateValidityChecker(si), obstacles_(obstacles), carSide_(carSide) {}
    
        bool isValid(const ob::State* state) const override {
            const auto* cs  = state->as<ob::CompoundState>();
            const auto* se2 = cs->as<ob::SE2StateSpace::StateType>(0);
    
            const double x     = se2->getX();
            const double y     = se2->getY();
            const double theta = se2->getYaw();
    
            return isValidSquare(x, y, theta, carSide_, obstacles_);
        }
    
    private:
        std::vector<Rectangle> obstacles_;
        double carSide_;
};

void makeStreet(std::vector<Rectangle> &  obstacles )
{
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
    
    Rectangle r5;
    r5.x=-25;r5.y=-45;
    r5.width=10;r5.height = 40;
    obstacles.push_back(r5);

    Rectangle r6;
    r6.x=-25;r6.y=5;
    r6.width=10;r6.height=40;
    obstacles.push_back(r6);

    Rectangle r7;
    r7.x = -5; r7.y = -25;
    r7.width = 10; r7.height = 70;
    obstacles.push_back(r7);

    Rectangle r8;
    r8.x = 25; r8.y = -5;
    r8.width = 20; r8.height=10;
    obstacles.push_back(r8);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle>& obstacles)
{
    namespace ob = ompl::base;
    namespace oc = ompl::control;

    // --- State space: SE(2) + R (velocity) ---
    // NOTE: SE2StateSpace already properly handles angle wrapping for theta!
    auto se2 = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds xyBounds(2);
    xyBounds.setLow(-50);
    xyBounds.setHigh(50);
    se2->setBounds(xyBounds);

    auto vspace = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds vBounds(1);
    vBounds.setLow(-10);
    vBounds.setHigh(10);
    vspace->setBounds(vBounds);

    auto space = std::make_shared<ob::CompoundStateSpace>();
    // FIX: Increase SE2 weight to prioritize spatial convergence
    // SE2 distance gets normalized by bounds (100 units), so we need higher weight
    space->addSubspace(se2, 10.0);    // Higher weight for position/orientation
    space->addSubspace(vspace, 0.5);  // Lower weight for velocity
    space->lock();

    // --- Control space: [omega, accel] ---
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds uBounds(2);
    uBounds.setLow(0, -2.0);
    uBounds.setHigh(0,  2.0);
    uBounds.setLow(1, -3.0);
    uBounds.setHigh(1,  3.0);
    cspace->setBounds(uBounds);

    // --- SimpleSetup + ODE propagator ---
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);
    auto si = ss->getSpaceInformation();

    // ODE solver / propagator (MUST be set BEFORE projection registration!)
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(si, &carODE);
    si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    
    // Set control duration and propagation step size
    si->setMinMaxControlDuration(1, 100);  // More steps for car (needs more control authority)
    si->setPropagationStepSize(0.02);      // 20ms integration step for car dynamics

    // Validity checker (use ss-> like pendulum to ensure consistency)
    ss->setStateValidityChecker(std::make_shared<CarValidityChecker>(si, obstacles, CAR_SIZE));

    // --- Start / Goal states ---
    ob::ScopedState<> start(space);
    start[0] = -40.0; start[1] = -40.0; start[2] = 0.0; start[3] = 0.0;
    
    // Goal position
    double goalX = 40.0, goalY = 40.0, goalTheta = 0.0, goalV = 0.0;
    
    // Use CUSTOM goal region with TRUE Euclidean distance, tolerance = 3.0
    auto goalRegion = std::make_shared<CarGoalRegion>(si, goalX, goalY, goalTheta, goalV, 3.0);
    
    ss->setStartState(start);
    ss->setGoal(goalRegion);
    
    std::cout << "Goal tolerance set to 3.0 meters (true Euclidean distance)" << std::endl;

    // Projection for KPIECE (register AFTER everything else is set up!)
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));

    return ss;
}


void planCar(ompl::control::SimpleSetupPtr & ss , int choice)
{
    ob::PlannerPtr planner;
    std::string plannerName;
    
    if (choice == 1) {
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        plannerName = "rrt";
        std::cout << "Using RRT planner" << std::endl;
    }    
    else if (choice == 2) {
        // FIX: Use default KPIECE1 parameters - they work when state space is correct!
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        plannerName = "kpiece";
        std::cout << "Using KPIECE1 planner with default parameters" << std::endl;
    } 
    else if (choice == 3) {
        auto rgrrtPlanner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
        rgrrtPlanner->setNumControlSamples(11);
        rgrrtPlanner->setReachabilityDuration(0.05);
        planner = rgrrtPlanner;
        plannerName = "rgrrt";
        std::cout << "Using RG-RRT planner" << std::endl;
    }

    ss->setPlanner(planner);
    ss->setup();
    
    // Set termination condition - only stop when goal is reached or timeout
    std::cout << "Starting planning with " << (choice == 2 ? "120" : "60") << " second timeout..." << std::endl;
    std::cout << "Planner will NOT terminate until goal (within 3.0m) is reached or timeout expires." << std::endl;
    
    // Increase timeout for KPIECE1 which may need more time with fixed weights
    double timeout = (choice == 2) ? 120.0 : 60.0;  // 120s for KPIECE1, 60s for others
    ob::PlannerStatus solved = ss->solve(timeout);
    
    if (solved) {
        std::cout << "\n=== SOLUTION FOUND ===" << std::endl;
        
        // Get the final state and check distance to goal
        oc::PathControl &path = ss->getSolutionPath();
        std::vector<ob::State*> &states = path.getStates();
        const ob::State* finalState = states.back();
        
        const auto *cs = finalState->as<ob::CompoundState>();
        const auto *se2 = cs->as<ob::SE2StateSpace::StateType>(0);
        const auto *vvec = cs->as<ob::RealVectorStateSpace::StateType>(1);
        
        double finalX = se2->getX();
        double finalY = se2->getY();
        double finalTheta = se2->getYaw();
        double finalV = vvec->values[0];
        
        double distToGoal = std::sqrt(std::pow(finalX - 40.0, 2) + std::pow(finalY - 40.0, 2));
        
        std::cout << "Final state: (" << finalX << ", " << finalY << ", " 
                  << finalTheta << " rad, " << finalV << " m/s)" << std::endl;
        std::cout << "Euclidean distance to goal (40, 40): " << distToGoal << " meters" << std::endl;
        std::cout << "Within tolerance (3.0m)? " << (distToGoal <= 3.0 ? "YES ✓" : "NO ✗") << std::endl;
        
        std::cout << "\nFound solution:" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
        
        std::string filename = "output/car_" + plannerName + "_path.txt";
        std::ofstream fout(filename);
        ss->getSolutionPath().printAsMatrix(fout);
        fout.close();
        
        std::cout << "Solution saved to " << filename << std::endl;
    } else {
        std::cout << "No solution found" << std::endl;
    }
}


void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // Create benchmark object
    ompl::tools::Benchmark b(*ss, "Car Benchmark");
    auto si = ss->getSpaceInformation();
    
    // Add RRT
    b.addPlanner(std::make_shared<ompl::control::RRT>(si));
    
    // FIX: Add KPIECE1 with DEFAULT parameters - no tuning needed!
    b.addPlanner(std::make_shared<ompl::control::KPIECE1>(si));
    
    // Add RG-RRT
    auto rgrrt = std::make_shared<ompl::control::RGRRT>(si);
    rgrrt->setNumControlSamples(11);
    rgrrt->setReachabilityDuration(0.05);
    b.addPlanner(rgrrt);
    
    // Set benchmark parameters
    ompl::tools::Benchmark::Request req;
    req.maxTime = 60.0;      // Reduced from 300s - should solve much faster now
    req.maxMem = 2000.0;
    req.runCount = 50;       // Increased to 50 runs for better statistics
    req.displayProgress = true;

    b.setPostRunEvent([si](const ompl::base::PlannerPtr& planner,
                        ompl::tools::Benchmark::RunProperties& run) {
        ompl::base::PlannerData pd(si);
        planner->getPlannerData(pd);
        run["tree_nodes INTEGER"] = std::to_string(pd.numVertices());
    });

    b.benchmark(req);
    b.saveResultsToFile("car_benchmark.log");
    std::cout << "Benchmark results saved to car_benchmark.log" << std::endl;
}

int main()
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark?" << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    auto ss = createCar(obstacles);

    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner?" << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;
            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    else if (choice == 2)
    {
        benchmarkCar(ss);
    }

    return 0;
}
