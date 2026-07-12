#pragma once
#include "IPathGeometry.h"
#include "ITrajectoryProfile.h"
#include "Quatf.h"

// Combines a path geometry with a scalar ITrajectoryProfile to produce a
// time-parameterised Cartesian move.
//
// The profile runs over arc length [0, path->getLength()] — it is completely
// unaware of whether the path is a line, arc, or anything else.
// Orientation is SLERPed from startOrientation to endOrientation by the
// arc-length fraction s / totalLength.
//
// IMPORTANT: accelTangential from evaluate() is the tangential component only.
// On curved paths (ArcPath) there is also centripetal acceleration
// (v² / radius, perpendicular to travel) that is NOT returned here.
// See CLAUDE.md architecture notes for details.
class CartesianMove {
public:
    // path and profile must outlive this object.
    bool plan(IPathGeometry*     path,
              const Quatf&       startOrientation,
              const Quatf&       endOrientation,
              ITrajectoryProfile* profile,
              const TrajectoryLimits& limits,
              float targetDuration = 0.0f);

    // Returns true while in motion, false once settled at the path end.
    bool evaluate(float t,
                  Vec3&  position,
                  Vec3&  velocity,
                  Vec3&  accelTangential,
                  Quatf& orientation) const;

    float getDuration() const;

private:
    IPathGeometry*      _path    = nullptr;
    ITrajectoryProfile* _profile = nullptr;
    Quatf _q0, _q1;
    float _length = 0.0f;
};
