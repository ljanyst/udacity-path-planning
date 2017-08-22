//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.08.2017
//------------------------------------------------------------------------------

#pragma once

#include "map.hh"
#include "utils.hh"

#include <memory>

//------------------------------------------------------------------------------
//! Predict paths of sensor fussion objects
//------------------------------------------------------------------------------
class PathPredictor {
  public:
    //--------------------------------------------------------------------------
    //! Constructor
    //--------------------------------------------------------------------------
    PathPredictor(const Map &map, double distance):
        map_(map), distance_(distance) {};

    //--------------------------------------------------------------------------
    //! Destructor
    //--------------------------------------------------------------------------
    virtual ~PathPredictor() {};

    //--------------------------------------------------------------------------
    //! Add sensor fussion objects
    //--------------------------------------------------------------------------
    void AddObjects(const std::vector<SFObject> &objects,
                    const Vehicle               &v);

    //--------------------------------------------------------------------------
    //! Find the leader wrt our vehicle
    //--------------------------------------------------------------------------
    std::shared_ptr<SFObject> FindLeader(const Vehicle &v, int lane = -1) const;

    //--------------------------------------------------------------------------
    //! Find the follower wrt our vehicle
    //--------------------------------------------------------------------------
    std::shared_ptr<SFObject> FindFollower(const Vehicle &v,
                                           int lane = -1) const;

  private:
    std::vector<SFObject>  objects_;
    const Map             &map_;
    double                 distance_;
};
