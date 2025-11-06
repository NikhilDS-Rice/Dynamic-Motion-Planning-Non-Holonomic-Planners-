///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Shrikant & Nikhil
//////////////////////////////////////

#include "RG-RRT.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <limits>

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates, "0,1");
    Planner::declareParam<unsigned int>("num_control_samples", this, &RGRRT::setNumControlSamples, &RGRRT::getNumControlSamples, "1:100");
    Planner::declareParam<double>("reachability_duration", this, &RGRRT::setReachabilityDuration, &RGRRT::getReachabilityDuration, "0.01:1.0");
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            // Free reachable set states
            for (auto &reachState : motion->reachableSet)
            {
                if (reachState)
                    si_->freeState(reachState);
            }
            delete motion;
        }
    }
}

void ompl::control::RGRRT::computeReachableSet(Motion *motion)
{
    // Clear any existing reachable set
    for (auto &reachState : motion->reachableSet)
    {
        if (reachState)
            si_->freeState(reachState);
    }
    motion->reachableSet.clear();

    // Assume control space is RealVectorControlSpace
    auto *controlSpace = siC_->getControlSpace()->as<RealVectorControlSpace>();
    const unsigned int controlDim = controlSpace->getDimension();
    
    // Get control bounds
    const ompl::base::RealVectorBounds &bounds = controlSpace->getBounds();
    
    // For each control dimension, sample evenly spaced values
    std::vector<std::vector<double>> controlSamplesPerDim;
    for (unsigned int d = 0; d < controlDim; ++d)
    {
        std::vector<double> samples;
        double lower = bounds.low[d];
        double upper = bounds.high[d];
        
        // Sample numControlSamples_ evenly spaced values for this dimension
        for (unsigned int i = 0; i < numControlSamples_; ++i)
        {
            double value = lower + (upper - lower) * i / (numControlSamples_ - 1.0);
            samples.push_back(value);
        }
        controlSamplesPerDim.push_back(samples);
    }
    
    // Generate all combinations of control samples
    // For 1D control (pendulum): just iterate through samples
    // For 2D control (car): iterate through all pairs
    
    if (controlDim == 1)
    {
        // Single control dimension (e.g., pendulum torque)
        for (unsigned int i = 0; i < numControlSamples_; ++i)
        {
            Control *ctrl = siC_->allocControl();
            double *ctrlValues = ctrl->as<RealVectorControlSpace::ControlType>()->values;
            ctrlValues[0] = controlSamplesPerDim[0][i];
            
            // Propagate from current state with this control
            base::State *reachedState = si_->allocState();
            unsigned int steps = siC_->propagateWhileValid(motion->state, ctrl, reachabilityDuration_, reachedState);
            
            // Check if the propagation was valid and we reached a valid state
            if (steps > 0 && si_->isValid(reachedState))
            {
                motion->reachableSet.push_back(reachedState);
            }
            else
            {
                si_->freeState(reachedState);
            }
            
            siC_->freeControl(ctrl);
        }
    }
    else if (controlDim == 2)
    {
        // Two control dimensions (e.g., car: angular velocity and acceleration)
        // Sample first dimension more densely, second dimension at extremes only to save computation
        for (unsigned int i = 0; i < numControlSamples_; ++i)
        {
            for (unsigned int j = 0; j < numControlSamples_; j += (numControlSamples_ - 1))  // Only extremes for 2nd dim
            {
                Control *ctrl = siC_->allocControl();
                double *ctrlValues = ctrl->as<RealVectorControlSpace::ControlType>()->values;
                ctrlValues[0] = controlSamplesPerDim[0][i];
                ctrlValues[1] = (j == 0) ? controlSamplesPerDim[1][0] : controlSamplesPerDim[1][numControlSamples_ - 1];
                
                // Propagate from current state with this control
                base::State *reachedState = si_->allocState();
                unsigned int steps = siC_->propagateWhileValid(motion->state, ctrl, reachabilityDuration_, reachedState);
                
                // Check if valid
                if (steps > 0 && si_->isValid(reachedState))
                {
                    motion->reachableSet.push_back(reachedState);
                }
                else
                {
                    si_->freeState(reachedState);
                }
                
                siC_->freeControl(ctrl);
            }
        }
    }
}

bool ompl::control::RGRRT::isReachableFromMotion(Motion *motion, const base::State *qrand)
{
    
    // If reachable set is empty, always accept (no information to reject)
    if (motion->reachableSet.empty())
        return true;
    
    double distNearToRand = si_->distance(motion->state, qrand);
    
    // Check if qnear is closer than ALL reachable states
    bool qnearIsClosest = true;
    for (const auto &reachState : motion->reachableSet)
    {
        double distReachToRand = si_->distance(reachState, qrand);
        
        // If ANY reachable state is closer than or equal to qnear, then qnear is NOT closest
        if (distReachToRand <= distNearToRand)
        {
            qnearIsClosest = false;
            break;
        }
    }
    
    // Accept if qnear is NOT the closest (i.e., some reachable state is closer)
    // Reject if qnear is the closest (shadows the region)
    return !qnearIsClosest;
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Add start states to the tree
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        
        // Compute reachable set for this start state
        computeReachableSet(motion);
        
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    unsigned int rejectedSamples = 0;
    unsigned int totalSamples = 0;

    while (ptc == false)
    {
        totalSamples++;
        
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        
        /* RG-RRT: Check if qrand is reachable from qnear */
        if (!isReachableFromMotion(nmotion, rstate))
        {
            // qnear is closer than all states in R(qnear), so discard qrand
            rejectedSamples++;
            continue;  // Sample a new random state
        }

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        if (addIntermediateStates_)
        {
            // Add intermediate states along the propagation
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    
                    // Compute reachable set for this new motion
                    computeReachableSet(motion);
                    
                    lastmotion = motion;
                    nn_->add(motion);
                    
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                // Compute reachable set for this new motion
                computeReachableSet(motion);

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states (rejected %u samples out of %u)", getName().c_str(), nn_->size(), rejectedSamples, totalSamples);

    return {solved, approximate};
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}

