///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "RG-RRT.h"

// TODO: Implement RGRRT as described
// This is a stub to allow compilation for the checkpoint
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <limits>
#include <iostream>

using namespace ompl;
using namespace ompl::control;

RG_RRT::RG_RRT(const SpaceInformationPtr &si)
    : base::Planner(si, "RG_RRT"), si_(si), deltaT_(0.1), numReachSamples_(11) {
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
}

void RG_RRT::setup() {
    base::Planner::setup();
    sampler_ = si_->allocStateSampler();
}

void RG_RRT::clear() { Planner::clear(); }

void RG_RRT::computeReachableSet(Motion *m) {
    m->reachable.clear();

    // Evenly spaced controls across bounds
    auto *cs = si_->getControlSpace()->as<RealVectorControlSpace>();
    auto bounds = cs->getBounds();
    double umin = bounds.low[0];
    double umax = bounds.high[0];
    for (unsigned int i = 0; i < numReachSamples_; ++i) {
        double val = umin + i * (umax - umin) / (numReachSamples_ - 1);
        Control *u = si_->allocControl();
        u->as<RealVectorControlSpace::ControlType>()->values[0] = val;

        base::State *r = si_->allocState();
        si_->propagate(m->state, u, deltaT_, r);

        if (si_->isValid(r))
            m->reachable.push_back(r);
    }
}

bool RG_RRT::isReachableCloser(const Motion *m, const base::State *qrand, base::State *&x_r_near) {
    double d_near = si_->distance(m->state, qrand);
    double best = d_near;
    x_r_near = nullptr;

    for (auto *r : m->reachable) {
        double d = si_->distance(r, qrand);
        if (d < best) {
            best = d;
            x_r_near = r;
        }
    }
    return x_r_near != nullptr;
}

base::PlannerStatus RG_RRT::solve(const base::PlannerTerminationCondition &ptc) {
    checkValidity();
    goal_ = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (!goal_) return base::PlannerStatus::INVALID_GOAL;

    auto *start = new Motion(si_);
    si_->copyState(start->state, pdef_->getStartState(0));
    computeReachableSet(start);

    std::vector<Motion *> tree;
    tree.push_back(start);

    base::State *qrand = si_->allocState();
    base::State *x_r_near = nullptr;

    while (!ptc) {
        sampler_->sampleUniform(qrand);

        // Find nearest node
        Motion *nearest = nullptr;
        double best = std::numeric_limits<double>::infinity();
        for (auto *m : tree) {
            double d = si_->distance(m->state, qrand);
            if (d < best) { best = d; nearest = m; }
        }

        if (!nearest) continue;

        // Discard sample if unreachable
        if (!isReachableCloser(nearest, qrand, x_r_near)) continue;

        auto *newMotion = new Motion(si_);
        si_->copyState(newMotion->state, x_r_near);
        newMotion->parent = nearest;
        computeReachableSet(newMotion);
        tree.push_back(newMotion);

        if (goal_->isSatisfied(newMotion->state)) {
            std::cout << "Goal reached!" << std::endl;
            // Construct path
            std::vector<base::State *> path;
            for (Motion *m = newMotion; m; m = m->parent)
                path.push_back(m->state);

            std::reverse(path.begin(), path.end());
            auto pathObj = std::make_shared<PathControl>(si_);
            for (size_t i = 0; i + 1 < path.size(); ++i)
                pathObj->append(path[i], path[i + 1], deltaT_);
            pdef_->addSolutionPath(base::PathPtr(pathObj), false, 0.0, getName());
            return base::PlannerStatus::EXACT_SOLUTION;
        }
    }

    return base::PlannerStatus::TIMEOUT;
}
