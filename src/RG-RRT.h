///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/PathControl.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#ifndef RGRRT_H
#define RGRRT_H

namespace ompl
{
    namespace control
    {
        // TODO: Implement RGRRT as described

        class RG_RRT : public base::Planner {
            public:
                RG_RRT(const SpaceInformationPtr &si);
            
                ~RG_RRT() override = default;
            
                void setup() override;
                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
                void clear() override;
            
            protected:
                struct Motion {
                    base::State *state;
                    Motion *parent;
                    std::vector<base::State *> reachable;  // reachable set R(q)
                    Control *control;
                    double duration;
            
                    Motion(const SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr), control(si->allocControl()), duration(0.0) {}
                    ~Motion() = default;
                };
            
                // Helper functions
                void computeReachableSet(Motion *m);
                bool isReachableCloser(const Motion *m, const base::State *qrand, base::State *&x_r_near);
            
                SpaceInformationPtr si_;
                base::StateSamplerPtr sampler_;
                base::GoalSampleableRegion *goal_;
                double deltaT_;
                unsigned int numReachSamples_;
            };

    }  // namespace control 
}  // namespace ompl

#endif
