#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
  class FullBodySnapMotionPrimitive : public MotionPrimitive {
    public:
      FullBodySnapMotionPrimitive(GoalStatePtr& goal){m_goal = goal;}
      virtual bool apply(const GraphState& graph_state, 
          GraphStatePtr& successor,
          TransitionData& t_data);
      virtual void print() const;
      virtual int motion_type() const { return MPrim_Types::FULLBODY_SNAP; }; 
      virtual void computeCost(const MotionPrimitiveParams& params);
      bool computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data);
      GoalStatePtr m_goal;
  };
  typedef boost::shared_ptr<FullBodySnapMotionPrimitive> FullBodySnapMotionPrimitivePtr;

}
