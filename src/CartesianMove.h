#pragma once
#include <IDevice.h>            // Universal-Device-Interface
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
//
// Derives from IDevice for the same reason TrajectoryGroup does, and with
// the same boundaries: it composes other objects and can fail in ways a
// bare bool can't explain, whereas ITrajectoryProfile/IPathGeometry stay
// plain math primitives. evaluate() is the hot path and is deliberately
// NOT virtual and unchanged; reportError() is only ever called from plan().
class CartesianMove : public IDevice {
public:
    enum Error {
        ERR_NONE = 0,
        ERR_NULL_PATH        = 1,  // path pointer was null
        ERR_NULL_PROFILE     = 2,  // profile pointer was null
        ERR_PROFILE_PLAN_FAILED = 3,  // the scalar profile's plan() returned false
    };

    // Same reasoning as TrajectoryGroup::Status -- "planned" vs "not
    // planned", never "moving": this class is stateless with respect to
    // time and evaluate() is const.
    enum Status {
        STATUS_NONE    = 0,  // no plan loaded (or the last plan failed)
        STATUS_PLANNED = 1,  // a valid plan is loaded
    };

    explicit CartesianMove(const char* name = "CartesianMove") : _name(name) {}

    // path and profile must outlive this object and must be non-null.
    //
    // Returns false and reports through the global error sink if either
    // pointer is null or the profile's own plan() fails. On failure the
    // move is left with NO plan loaded, so evaluate() is inert rather than
    // dereferencing a half-configured move. (Before this, a null path or
    // profile dereferenced immediately, and the profile's plan() result was
    // returned but the move was still left "configured".)
    bool plan(IPathGeometry*     path,
              const Quatf&       startOrientation,
              const Quatf&       endOrientation,
              ITrajectoryProfile* profile,
              const TrajectoryLimits& limits,
              float targetDuration = 0.0f);

    // Returns true while in motion, false once settled at the path end.
    // Inert (returns false, leaves outputs untouched) when no plan is
    // loaded. Hot path: unchanged, non-virtual, allocation-free.
    bool evaluate(float t,
                  Vec3&  position,
                  Vec3&  velocity,
                  Vec3&  accelTangential,
                  Quatf& orientation) const;

    float getDuration() const;

    // ------------------------------------------------------------
    // IDevice
    // ------------------------------------------------------------
    bool begin() override;                    // trivially true; clears any latched error
    bool isOnline() const override { return true; }   // pure computation -- see TrajectoryGroup.h
    DeviceState getState() const override;    // ERRORED / IDLE -- never BUSY
    uint32_t    getStatus() const override;
    uint32_t    getError()  const override;
    const char* getStatusString(uint32_t status) const override;
    const char* getErrorString(uint32_t err) const override;
    const char* getDeviceName() const override { return _name; }

private:
    void setError(uint32_t err);
    void invalidatePlan();

    const char* _name;
    IPathGeometry*      _path    = nullptr;
    ITrajectoryProfile* _profile = nullptr;
    Quatf _q0, _q1;
    float _length = 0.0f;
    uint32_t _error = ERR_NONE;
};
