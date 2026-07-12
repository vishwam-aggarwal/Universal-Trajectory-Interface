#pragma once
#include "ITrajectoryProfile.h"

// Coordinates up to MAX_AXES independent scalar profiles so they all arrive
// at their targets simultaneously (joint-space synchronisation).
//
// plan() first finds each axis's minimum-time duration, takes the longest,
// then re-plans every axis with that shared targetDuration.  Axes with
// zero distance are unaffected (they settle immediately regardless).
class TrajectoryGroup {
public:
    static const int MAX_AXES = 6;

    // profiles[], q0[], qf[], and limits[] must each have at least 'count'
    // elements. count must be in [1, MAX_AXES]. Profile pointers must outlive
    // this object.
    bool plan(ITrajectoryProfile** profiles,
              const float*         q0,
              const float*         qf,
              const TrajectoryLimits* limits,
              int count);

    // pos[], vel[], accel[] must each have at least getCount() elements.
    // Returns true while any axis is still in motion.
    bool evaluate(float t, float* pos, float* vel, float* accel) const;

    float getDuration() const;
    int   getCount()    const;

private:
    ITrajectoryProfile* _profiles[MAX_AXES] = {};
    int   _count    = 0;
    float _duration = 0.0f;
};
