#pragma once
#include <IDevice.h>            // Universal-Device-Interface
#include "ITrajectoryProfile.h"

// Coordinates up to MAX_AXES independent scalar profiles so they all arrive
// at their targets simultaneously (joint-space synchronisation).
//
// plan() first finds each axis's minimum-time duration, takes the longest,
// then re-plans every axis with that shared targetDuration.  Axes with
// zero distance are unaffected (they settle immediately regardless).
//
// ------------------------------------------------------------------
// Why this derives from IDevice but ITrajectoryProfile does not
// ------------------------------------------------------------------
// ITrajectoryProfile/TrapezoidalProfile are stateless per-axis math
// primitives: plan()'s bool return already says everything there is to
// say, and there is no fault mode beyond it. Forcing IDevice onto them
// would add the most boilerplate for the least value anywhere in the
// family. TrajectoryGroup is different -- it *coordinates* several of
// them, so it can fail in ways a caller genuinely wants explained
// ("which axis? why?"), and a bare bool can't carry that. See getError().
//
// Real-time note: IDevice adds a vtable pointer per object and nothing
// else (its sink storage is static). evaluate() -- the only hot-path
// method -- is deliberately NOT virtual and is unchanged, so the cyclic
// path stays allocation-free, exception-free and free of new dynamic
// dispatch. reportError() is only ever called from plan(), never from
// evaluate().
class TrajectoryGroup : public IDevice {
public:
    static const int MAX_AXES = 6;

    enum Error {
        ERR_NONE = 0,
        ERR_INVALID_AXIS_COUNT = 1,  // count outside [1, MAX_AXES]
        ERR_NULL_PROFILE       = 2,  // a profiles[i] entry was null
        ERR_AXIS_PLAN_FAILED   = 3,  // an axis's own plan() returned false
    };

    // Status tier: whether a usable plan is currently loaded. Deliberately
    // NOT "moving" -- this class is stateless with respect to time (the
    // caller supplies t to evaluate(), which is const and remembers
    // nothing), so it cannot honestly know whether a move is in progress.
    // That's why getState() never reports BUSY; see below.
    enum Status {
        STATUS_NONE    = 0,  // no plan loaded (or the last plan failed)
        STATUS_PLANNED = 1,  // a valid plan is loaded; getDuration() is meaningful
    };

    // name is what getDeviceName() (and the error sink's sourceName)
    // reports. Must outlive this object; a string literal is the normal
    // case. Defaulted so existing `TrajectoryGroup g;` declarations --
    // including MotionDevice's member -- keep compiling unchanged.
    explicit TrajectoryGroup(const char* name = "TrajectoryGroup") : _name(name) {}

    // profiles[], q0[], qf[], and limits[] must each have at least 'count'
    // elements. count must be in [1, MAX_AXES]. Profile pointers must outlive
    // this object and must be non-null.
    //
    // Returns false and reports through the global error sink (see
    // getError()) if the axis count is out of range, any profile pointer is
    // null, or any axis's own plan() fails -- e.g. TrapezoidalProfile
    // rejects a non-positive or NaN vMax/aMax. On any failure the group is
    // left with NO plan loaded, so a subsequent evaluate() is inert rather
    // than acting on a half-planned group.
    bool plan(ITrajectoryProfile** profiles,
              const float*         q0,
              const float*         qf,
              const TrajectoryLimits* limits,
              int count);

    // pos[], vel[], accel[] must each have at least getCount() elements.
    // Returns true while any axis is still in motion.
    // Hot path: unchanged, non-virtual, allocation-free.
    bool evaluate(float t, float* pos, float* vel, float* accel) const;

    float getDuration() const;
    int   getCount()    const;

    // ------------------------------------------------------------
    // IDevice
    // ------------------------------------------------------------
    // Trivially true: a planner has no hardware to bring up. Clears any
    // latched error, so begin() doubles as the "reset this planner" call.
    bool begin() override;

    // Always true. This is pure computation with no hardware and no
    // bring-up step, so there is nothing it could be offline *from* --
    // reporting OFFLINE would be inventing a state this class cannot
    // actually enter. (Tying it to begin() would also make every
    // TrajectoryGroup owned as a plain member -- MotionDevice's, for one --
    // report OFFLINE forever, since such owners never call begin() on it.)
    bool isOnline() const override { return true; }

    // ERRORED after a failed plan(), IDLE otherwise. **Never BUSY**: see
    // the Status enum above -- a time-stateless planner cannot honestly
    // claim to know whether a move is in progress, and faking it would
    // break the family's never-fake-it rule.
    DeviceState getState() const override;

    uint32_t    getStatus() const override;
    uint32_t    getError()  const override;
    const char* getStatusString(uint32_t status) const override;
    const char* getErrorString(uint32_t err) const override;
    const char* getDeviceName() const override { return _name; }

private:
    // Sets _error and reports it through the inherited global sink.
    void setError(uint32_t err);

    // Drops any loaded plan, so evaluate() is inert after a failure.
    void invalidatePlan();

    const char* _name;
    ITrajectoryProfile* _profiles[MAX_AXES] = {};
    int   _count    = 0;
    float _duration = 0.0f;
    uint32_t _error = ERR_NONE;
};
