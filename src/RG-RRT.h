///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Shrikant & Nikhil
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/control/SpaceInformation.h>
#include <vector>

namespace ompl
{
    namespace control
    {
        
        /** \brief Reachability-Guided Rapidly-exploring Random Tree */
        class RGRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            RGRRT(const SpaceInformationPtr &si);

            ~RGRRT() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** \brief Set the goal bias
                This parameter controls the likelihood of the planner
                selecting the goal state as a sample. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set whether intermediate states are added during propagation */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief Get whether intermediate states are added during propagation */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Set the number of controls to sample for reachability approximation */
            void setNumControlSamples(unsigned int numSamples)
            {
                numControlSamples_ = numSamples;
            }

            /** \brief Get the number of controls sampled for reachability */
            unsigned int getNumControlSamples() const
            {
                return numControlSamples_;
            }

            /** \brief Set the duration for reachability propagation */
            void setReachabilityDuration(double duration)
            {
                reachabilityDuration_ = duration;
            }

            /** \brief Get the reachability propagation duration */
            double getReachabilityDuration() const
            {
                return reachabilityDuration_;
            }

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion with reachable set
                This extends the basic motion to also store an approximation
                of the reachable set from this state */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief Approximation of reachable set: states reachable from this motion */
                std::vector<base::State *> reachableSet;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute reachable set for a motion */
            void computeReachableSet(Motion *motion);

            /** \brief Check if random state is reachable from nearest motion
                Returns true if qnear is closer to qrand than any state in R(qnear) */
            bool isReachableFromMotion(Motion *motion, const base::State *qrand);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief Flag indicating whether intermediate states are added to the tree or only the final state */
            bool addIntermediateStates_{false};

            /** \brief Number of controls to sample for reachability set approximation */
            unsigned int numControlSamples_{11};

            /** \brief Duration for reachability propagation (small time step) */
            double reachabilityDuration_{0.05};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion. Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };

    }  // namespace control 
}  // namespace ompl

#endif
