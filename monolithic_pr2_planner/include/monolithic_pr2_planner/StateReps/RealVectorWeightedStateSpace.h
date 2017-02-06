#pragma once
#include <ros/ros.h>
#include <angles/angles.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/base/goals/GoalState.h>


namespace ompl
{
    namespace base
    {

        class RealVectorWeightedStateSpace : public ompl::base::RealVectorStateSpace
        {
            public:

               class StateType : public State
               {
                    public:
                        StateType(): State()
                        {

                        }

                        double operator[](unsigned int i) const
                        {
                            return values[i];
                        }

                        double & operator[](unsigned int i)
                        {
                            return values[i];
                        }

                        double *values;
               };

               RealVectorWeightedStateSpace(unsigned int dim, std::vector<double> weights) : RealVectorStateSpace(dim), weights_(weights.size(),0)
               {
                    type_ = STATE_SPACE_REAL_VECTOR;
                    setName("RealVectorWeighted" + getName());
                    dimensionNames_.resize(dim, "");
                    weights_ = weights;
               }

               ~RealVectorWeightedStateSpace()
               {

               }


               virtual double distance(const State *parent_comp, const State *child_comp) const
               {

                    double dist = weights_[0]*sqrt( sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[0], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[0]) + 
                                  sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[1], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[1]) +
                                  sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[2], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[2]) +
                                  sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[8], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[8])
                                  ) 
                                +                                  
                                  weights_[1]*( fabs(angles::shortest_angular_distance(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[3], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[3]) ) +
                                  fabs(angles::shortest_angular_distance(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[4], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[4]) ) +
                                  fabs(angles::shortest_angular_distance(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[5], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[5]) ) +
                                  fabs(angles::shortest_angular_distance(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[6], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[6]) ) +
                                  fabs(angles::shortest_angular_distance(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[7], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[7]) ) 
                                  ) 
                                +
                                  weights_[0]*sqrt( sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[9], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[9]) +
                                  sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[10], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[10])
                                  )
                                +
                                  weights_[1]*( fabs(angles::shortest_angular_distance(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[11], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[11]) ) 
                                  );



                    /*
                    const auto &parent_values = parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values;
                    const auto &child_values = parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values;
                    double diff_values[8];
                    for (size_t ii = 0; ii < 8; ++ii) {
                      diff_values[ii] = fabs(shortest_angular_distance(parent_values[ii], child_values[ii]));
                    }
                    double max_joint_angle = std::max(diff_values[6], diff_values[7]);
                    //max_joint_angle = std::max(max_joint_angle, diff_values[5]);
                    //max_joint_angle = std::max(max_joint_angle, diff_values[6]);
                    //max_joint_angle = std::max(max_joint_angle, diff_values[7]);
                    double new_dist = weights_[0]*sqrt( sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[0], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[0]) + 
                                  sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[1], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[1]) +
                                  sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[2], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[2]) +
                                  sqrdiff(parent_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[8], child_comp->as<ompl::base::RealVectorWeightedStateSpace::StateType>()->values[8]) ) 
                                +                                  
                                  weights_[1]*( max_joint_angle);
                    return new_dist;
                    */
                   return dist;
                   

               }

               double sqrdiff(double val1, double val2) const
               {
                    return (val1 - val2)*(val1 - val2);
               }

            private:

                std::vector<double> weights_;

        };
    }
}
