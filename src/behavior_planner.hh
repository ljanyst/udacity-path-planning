//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   10.08.2017
//------------------------------------------------------------------------------

#pragma once

#include "map.hh"
#include "path_predictor.hh"

#include <map>
#include <vector>
#include <functional>
#include <string>

//------------------------------------------------------------------------------
//! Plan definitions
//------------------------------------------------------------------------------
enum class Plan {
  kStartUp,
  kKeepLane,
  kChangeLeft,
  kChangeRight
};

//------------------------------------------------------------------------------
//! Behavior Planner
//------------------------------------------------------------------------------
class BehaviorPlanner {
  public:
    //--------------------------------------------------------------------------
    //! Constructor
    //--------------------------------------------------------------------------
    BehaviorPlanner(const Map &map, PathPredictor &predictor, double speed);

    //--------------------------------------------------------------------------
    //! Destructor
    //--------------------------------------------------------------------------
    virtual ~BehaviorPlanner() {}

    //--------------------------------------------------------------------------
    //! Plan the behavior
    //--------------------------------------------------------------------------
    Plan ComputeBehavior(const Vehicle &v);

    //--------------------------------------------------------------------------
    //! Convert plan to string
    //--------------------------------------------------------------------------
    static std::string PlanToString(Plan plan);

  private:
    //--------------------------------------------------------------------------
    // Helper type declarations
    //--------------------------------------------------------------------------
    using CostFunction     = std::function<double()>;
    using StateTransitions = std::map<Plan, CostFunction>;
    using StateMachine     = std::map<Plan, StateTransitions>;

    //--------------------------------------------------------------------------
    // Params for cost functions
    //--------------------------------------------------------------------------
    const Vehicle *v_;
    int            start_up1_ = 0;
    int            start_up2_ = 0;
    PathPredictor &predictor_;

    //--------------------------------------------------------------------------
    // State transition costs
    //--------------------------------------------------------------------------
    double SU2SU();
    double SU2KL();
    double KL2KL();
    double KL2CL(int direction);
    double CL2KL();

    //--------------------------------------------------------------------------
    // Private data members
    //--------------------------------------------------------------------------
    const Map    &map_;
    double        speed_;
    Plan          current_plan_ = Plan::kStartUp;
    StateMachine  sm_;
};
