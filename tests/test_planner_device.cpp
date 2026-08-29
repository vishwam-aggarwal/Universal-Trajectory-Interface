#include <cstdio>
#include <cstring>
#include <math.h>
#include "TrajectoryGroup.h"
#include "TrapezoidalProfile.h"
#include "CartesianMove.h"
#include "LinePath.h"

// ==================================================================
// IDevice conformance for the two planners, plus regression coverage for
// the plan() bugs fixed alongside the retrofit:
//   * TrajectoryGroup::plan() ignored every per-axis plan() return value,
//     so it reported success with an axis that never planned.
//   * CartesianMove::plan() dereferenced its path/profile arguments with
//     no null check.
// ==================================================================

static int s_passed = 0, s_failed = 0;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); ++s_passed; }
    else       { printf("  FAIL  %s\n", label); ++s_failed; }
}
static bool streq(const char* a, const char* b) {
    return a != nullptr && b != nullptr && strcmp(a, b) == 0;
}

struct SinkLog {
    int count = 0;
    const char* layer  = nullptr;
    const char* source = nullptr;
    uint32_t    code   = 0;
    const char* str    = nullptr;
};
static void collectSink(const char* layer, const char* source, uint32_t code,
                        const char* str, void* ctx) {
    SinkLog* log = static_cast<SinkLog*>(ctx);
    ++log->count;
    log->layer = layer; log->source = source; log->code = code; log->str = str;
}

int main() {
    printf("=== TrajectoryGroup / CartesianMove as IDevices ===\n\n");

    // ------------------------------------------------------------------
    // 1. TrajectoryGroup: healthy plan
    // ------------------------------------------------------------------
    {
        printf("-- 1. TrajectoryGroup healthy plan --\n");
        SinkLog log;
        IDevice::setGlobalErrorSink(collectSink, &log);

        TrajectoryGroup group("jointGroup");
        IDevice* dev = &group;

        check(streq(dev->getDeviceName(), "jointGroup"), "getDeviceName() from the ctor label");
        check(dev->isOnline(), "always online (pure computation, nothing to be offline from)");
        check(dev->getState() == DeviceState::IDLE, "IDLE before any plan");
        check(dev->getStatus() == TrajectoryGroup::STATUS_NONE, "STATUS_NONE with no plan");
        check(streq(dev->getStatusString(dev->getStatus()), "No plan loaded"), "status string");
        check(dev->begin(), "begin() is trivially true");

        TrapezoidalProfile p0, p1;
        ITrajectoryProfile* profiles[] = { &p0, &p1 };
        float q0[] = { 0.0f, 0.0f };
        float qf[] = { 10.0f, 5.0f };
        TrajectoryLimits lim[2];
        lim[0].vMax = 10.0f; lim[0].aMax = 10.0f;
        lim[1].vMax = 10.0f; lim[1].aMax = 10.0f;

        check(group.plan(profiles, q0, qf, lim, 2), "plan() succeeds");
        check(log.count == 0, "nothing reported on a healthy plan");
        check(dev->getState() == DeviceState::IDLE, "still IDLE after planning");
        check(dev->getStatus() == TrajectoryGroup::STATUS_PLANNED, "STATUS_PLANNED after planning");
        check(group.getCount() == 2 && group.getDuration() > 0.0f, "count/duration set");

        // Never BUSY, even mid-move: the group is stateless w.r.t. time.
        float pos[2], vel[2], acc[2];
        bool moving = group.evaluate(group.getDuration() * 0.5f, pos, vel, acc);
        check(moving, "evaluate() reports motion at mid-duration");
        check(dev->getState() == DeviceState::IDLE,
              "still IDLE mid-move -- a time-stateless planner never claims BUSY");

        IDevice::setGlobalErrorSink(nullptr);
    }

    // ------------------------------------------------------------------
    // 2. REGRESSION: a failing axis must fail the whole group
    // ------------------------------------------------------------------
    {
        printf("\n-- 2. regression: axis plan() failure is no longer swallowed --\n");
        SinkLog log;
        IDevice::setGlobalErrorSink(collectSink, &log);

        TrajectoryGroup group;
        TrapezoidalProfile p0, p1;
        ITrajectoryProfile* profiles[] = { &p0, &p1 };
        float q0[] = { 0.0f, 0.0f };
        float qf[] = { 10.0f, 5.0f };

        // Axis 1 gets a zero vMax -- TrapezoidalProfile::plan() returns false
        // and leaves that axis parked. Before the fix, the group returned
        // true here and axis 1 silently never moved.
        TrajectoryLimits lim[2];
        lim[0].vMax = 10.0f; lim[0].aMax = 10.0f;
        lim[1].vMax =  0.0f; lim[1].aMax = 10.0f;

        check(!group.plan(profiles, q0, qf, lim, 2),
              "plan() returns FALSE when an axis fails (was true before the fix)");
        check(group.getError() == TrajectoryGroup::ERR_AXIS_PLAN_FAILED, "ERR_AXIS_PLAN_FAILED");
        check(group.getState() == DeviceState::ERRORED, "state ERRORED");
        check(log.count == 1, "reported once through the sink");
        check(streq(log.layer, "TrajectoryGroup"), "layer == TrajectoryGroup");
        check(streq(log.source, "TrajectoryGroup"), "sourceName == default device name");
        check(streq(log.str, group.getErrorString(log.code)), "errorString matches");

        // And the group is left inert, not half-planned.
        check(group.getCount() == 0 && group.getStatus() == TrajectoryGroup::STATUS_NONE,
              "failed plan leaves NO plan loaded");
        float pos[2] = {9, 9}, vel[2] = {9, 9}, acc[2] = {9, 9};
        check(!group.evaluate(0.1f, pos, vel, acc), "evaluate() is inert after a failed plan");

        // A NaN limit is the same class of failure (NaN fails every compare).
        TrajectoryGroup g2;
        TrajectoryLimits nanLim[2];
        nanLim[0].vMax = 10.0f;   nanLim[0].aMax = 10.0f;
        nanLim[1].vMax = NAN;     nanLim[1].aMax = 10.0f;
        check(!g2.plan(profiles, q0, qf, nanLim, 2), "a NaN vMax also fails the group");

        group.begin();
        check(group.getState() == DeviceState::IDLE, "begin() clears the latched error");
        IDevice::setGlobalErrorSink(nullptr);
    }

    // ------------------------------------------------------------------
    // 3. TrajectoryGroup argument validation
    // ------------------------------------------------------------------
    {
        printf("\n-- 3. TrajectoryGroup argument validation --\n");
        SinkLog log;
        IDevice::setGlobalErrorSink(collectSink, &log);

        TrajectoryGroup group;
        TrapezoidalProfile p0;
        ITrajectoryProfile* profiles[] = { &p0 };
        ITrajectoryProfile* withNull[] = { &p0, nullptr };
        float q0[] = { 0.0f, 0.0f };
        float qf[] = { 1.0f, 1.0f };
        TrajectoryLimits lim[2];
        lim[0].vMax = 1.0f; lim[0].aMax = 1.0f;
        lim[1].vMax = 1.0f; lim[1].aMax = 1.0f;

        check(!group.plan(profiles, q0, qf, lim, 0), "count 0 rejected");
        check(group.getError() == TrajectoryGroup::ERR_INVALID_AXIS_COUNT, "ERR_INVALID_AXIS_COUNT");
        check(!group.plan(profiles, q0, qf, lim, TrajectoryGroup::MAX_AXES + 1), "count > MAX_AXES rejected");

        check(!group.plan(withNull, q0, qf, lim, 2), "null profile pointer rejected, not dereferenced");
        check(group.getError() == TrajectoryGroup::ERR_NULL_PROFILE, "ERR_NULL_PROFILE");
        check(log.count == 3, "each rejection reported exactly once");
        IDevice::setGlobalErrorSink(nullptr);
    }

    printf("\n=== CartesianMove ===\n\n");

    // ------------------------------------------------------------------
    // 4. CartesianMove: healthy plan + IDevice surface
    // ------------------------------------------------------------------
    {
        printf("-- 4. CartesianMove healthy plan --\n");
        SinkLog log;
        IDevice::setGlobalErrorSink(collectSink, &log);

        LinePath path(Vec3(0, 0, 0), Vec3(1, 0, 0));
        TrapezoidalProfile profile;
        TrajectoryLimits lim;
        lim.vMax = 1.0f; lim.aMax = 1.0f;
        CartesianMove move("toolMove");
        IDevice* dev = &move;

        check(streq(dev->getDeviceName(), "toolMove"), "getDeviceName() from the ctor label");
        check(dev->getStatus() == CartesianMove::STATUS_NONE, "STATUS_NONE before planning");
        check(dev->getState() == DeviceState::IDLE, "IDLE before planning");

        check(move.plan(&path, Quatf(), Quatf(), &profile, lim), "plan() succeeds");
        check(log.count == 0, "nothing reported on a healthy plan");
        check(dev->getStatus() == CartesianMove::STATUS_PLANNED, "STATUS_PLANNED after planning");
        check(move.getDuration() > 0.0f, "duration is set");

        Vec3 p, v, a; Quatf o;
        check(move.evaluate(move.getDuration() * 0.5f, p, v, a, o), "evaluate() reports motion mid-move");
        check(dev->getState() == DeviceState::IDLE, "still IDLE mid-move -- never BUSY");
        IDevice::setGlobalErrorSink(nullptr);
    }

    // ------------------------------------------------------------------
    // 5. REGRESSION: CartesianMove null arguments used to crash
    // ------------------------------------------------------------------
    {
        printf("\n-- 5. regression: CartesianMove null-argument handling --\n");
        SinkLog log;
        IDevice::setGlobalErrorSink(collectSink, &log);

        LinePath path(Vec3(0, 0, 0), Vec3(1, 0, 0));
        TrapezoidalProfile profile;
        TrajectoryLimits lim;
        lim.vMax = 1.0f; lim.aMax = 1.0f;

        CartesianMove m1;
        check(!m1.plan(nullptr, Quatf(), Quatf(), &profile, lim),
              "null path rejected, not dereferenced (was a crash)");
        check(m1.getError() == CartesianMove::ERR_NULL_PATH, "ERR_NULL_PATH");
        check(m1.getState() == DeviceState::ERRORED, "state ERRORED");

        CartesianMove m2;
        check(!m2.plan(&path, Quatf(), Quatf(), nullptr, lim),
              "null profile rejected, not dereferenced (was a crash)");
        check(m2.getError() == CartesianMove::ERR_NULL_PROFILE, "ERR_NULL_PROFILE");

        // A failed plan leaves the move inert rather than half-configured.
        Vec3 p, v, a; Quatf o;
        check(!m1.evaluate(0.1f, p, v, a, o), "evaluate() inert after a failed plan");
        check(m1.getDuration() == 0.0f, "getDuration() 0 with no plan");

        CartesianMove m3;
        TrajectoryLimits badLim;
        badLim.vMax = 0.0f; badLim.aMax = 1.0f;
        check(!m3.plan(&path, Quatf(), Quatf(), &profile, badLim), "bad limits fail the move");
        check(m3.getError() == CartesianMove::ERR_PROFILE_PLAN_FAILED, "ERR_PROFILE_PLAN_FAILED");
        check(streq(log.layer, "CartesianMove"), "reports under layer CartesianMove");
        check(log.count == 3, "three failures, three reports");
        IDevice::setGlobalErrorSink(nullptr);
    }

    // ------------------------------------------------------------------
    // 6. Both planners in one mixed IDevice* list, one sink
    // ------------------------------------------------------------------
    {
        printf("\n-- 6. mixed IDevice* list --\n");
        SinkLog log;
        IDevice::setGlobalErrorSink(collectSink, &log);

        TrajectoryGroup group("axes");
        CartesianMove   move("cart");
        IDevice* devices[] = { &group, &move };
        bool allIdle = true, allOnline = true;
        for (IDevice* d : devices) {
            if (d->getState() != DeviceState::IDLE) allIdle = false;
            if (!d->isOnline()) allOnline = false;
            d->update();   // inherited no-op: planners have no servicing step
        }
        check(allOnline && allIdle, "both report online/IDLE through IDevice*");
        check(streq(deviceStateToString(group.getState()), "IDLE"), "deviceStateToString()");

        // One registration, both layers distinguishable by layer + source.
        TrapezoidalProfile p;
        ITrajectoryProfile* profs[] = { &p };
        float q0[] = {0.0f}, qf[] = {1.0f};
        TrajectoryLimits bad[1];
        bad[0].vMax = 0.0f; bad[0].aMax = 1.0f;
        group.plan(profs, q0, qf, bad, 1);
        check(log.count == 1 && streq(log.source, "axes"), "group fault tagged source=axes");
        move.plan(nullptr, Quatf(), Quatf(), &p, bad[0]);
        check(log.count == 2 && streq(log.source, "cart"), "move fault tagged source=cart, same sink");

        IDevice::setGlobalErrorSink(nullptr);
    }

    printf("\n%d passed, %d failed\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
