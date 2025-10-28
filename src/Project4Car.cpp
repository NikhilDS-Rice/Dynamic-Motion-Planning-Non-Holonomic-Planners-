///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
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

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

constexpr double CAR_SIZE = 1.5; 

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd> projection ) const override
    {
        // TODO: Your projection for the car
        const auto *cs = state->as<ob::CompoundState>();
        const auto *se2 = cs->as<ob::SE2StateSpace::StateType>(0);
        projection[0] = se2->getX();
        projection[1] = se2->getY();
    }
};

void carODE(const ompl::control::ODESolver::StateType &  q , const ompl::control::Control * control ,
            ompl::control::ODESolver::StateType & qdot )
{
    // TODO: Fill in the ODE for the car's dynamics
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
            // State is Compound: [ SE2 (idx 0) | velocity (idx 1) ]
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
    // TODO: Fill in the vector of rectangles with your street environment.
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
    auto se2 = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds xyBounds(2);
    xyBounds.setLow(-50);   // x,y lower bound
    xyBounds.setHigh(50);   // x,y upper bound
    se2->setBounds(xyBounds);

    auto vspace = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds vBounds(1);
    vBounds.setLow(-10);    // can reverse
    vBounds.setHigh(10);    // forward limit
    vspace->setBounds(vBounds);

    auto space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(se2, 1.0);
    space->addSubspace(vspace, 0.5);
    space->lock();

    // --- Control space: [omega, accel] ---
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds uBounds(2);
    uBounds.setLow(0, -2.0);  // omega min
    uBounds.setHigh(0,  2.0); // omega max
    uBounds.setLow(1, -3.0);  // accel min
    uBounds.setHigh(1,  3.0); // accel max
    cspace->setBounds(uBounds);

    // --- SimpleSetup + ODE propagator ---
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);
    auto si = ss->getSpaceInformation();

    // Projection for KPIECE, etc.
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));

    // ODE solver / propagator
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(si, &carODE);
    si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    si->setMinMaxControlDuration(1, 20);
    si->setPropagationStepSize(0.05);

    // Validity checker (uses isValidSquare via CarValidityChecker)
    si->setStateValidityChecker(std::make_shared<CarValidityChecker>(si, obstacles, CAR_SIZE));

    // --- Start / Goal states: [x, y, theta, v] ---
    ob::ScopedState<> start(space), goal(space);
    start[0] = -40.0; start[1] = -40.0; start[2] = 0.0; start[3] = 0.0;
    goal[0]  =  40.0; goal[1]  =  40.0; goal[2]  = 0.0; goal[3]  = 0.0;

    ss->setStartAndGoalStates(start, goal, 2.0);  // goal tolerance

    return ss;
}

static void extractCarCSV(const oc::SimpleSetupPtr &ss, const std::string &fname, unsigned int interpolateN = 600) {
    auto gpath = ss->getSolutionPath().asGeometric();
    if (interpolateN > gpath.getStateCount())
        gpath.interpolate(interpolateN);

    std::ofstream out(fname);
    out << "x,y,theta,v\n";
    for (std::size_t i = 0; i < gpath.getStateCount(); ++i) {
        const auto *cs   = gpath.getState(i)->as<ob::CompoundState>();
        const auto *se2  = cs->as<ob::SE2StateSpace::StateType>(0);
        const auto *vSt  = cs->as<ob::RealVectorStateSpace::StateType>(1);
        out << se2->getX() << "," << se2->getY() << "," << se2->getYaw() << "," << vSt->values[0] << "\n";
    }
}

void planCar(ompl::control::SimpleSetupPtr & ss , int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    ob::PlannerPtr planner;
    std::string plannerName;
    if (choice == 1)   {
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        plannerName = "rrt";
    }    
    else if (choice == 2) {
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        plannerName = "kpiece";
    } 
    // else if (choice == 3)  planner = std::make_shared<oc::RG_RRT>(ss->getSpaceInformation());

    ss->setPlanner(planner);
    if (ss->solve(5.0)) {
        std::cout << "Found solution:" << std::endl;
        
        // Print the path to screen
        ss->getSolutionPath().printAsMatrix(std::cout);
        
        // Save the solution path to a file with unique name
        std::string filename = "output/car_" + plannerName + "_path.txt";
        std::ofstream fout(filename);
        ss->getSolutionPath().printAsMatrix(fout);
        fout.close();
        
        std::cout << "Solution saved to " << filename << std::endl;
    } else {
        std::cout << "No solution found\n";
    }
}


void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int  argc , char ** argv )
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
