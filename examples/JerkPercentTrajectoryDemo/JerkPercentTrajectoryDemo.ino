// JerkPercentTrajectoryDemo
//
// Drives one simulated motor axis through a jerk-percent S-curve. Same
// structure as SimpleTrajectoryDemo and SCurveTrajectoryDemo, with
// JerkPercentProfile in place of TrapezoidalProfile / SCurveProfile -- no
// real hardware is required, since SimulatedMotor just remembers the
// commanded position. Compiles and runs identically on any AVR, SAMD, or
// Teensy board with nothing wired up.
//
// All three demos run the SAME move (0 -> 90) with the SAME nominal limits
// (vMax 90, aMax 180), so they can be compared directly:
//
//   sketch                    duration   peak accel
//   SimpleTrajectoryDemo        1.50 s   180  (steps instantly)
//   SCurveTrajectoryDemo        1.75 s   180  (ramps at jMax)
//   JerkPercentTrajectoryDemo   1.50 s   269  (ramps, but faster)
//
// That is the whole point of this profile. SCurveProfile buys smooth
// acceleration by taking longer than the trapezoid. This one keeps the
// trapezoid's duration exactly and buys smoothness with acceleration
// instead -- it ramps harder over a shorter ramp.
//
// !! Peak acceleration EXCEEDS the nominal aMax you pass in !!
// The derived value is 2*aMax/(2 - jerkPercent/100): 1.49x nominal at 66%,
// and 2x at 100%. So aMax here is a nominal figure, not a hard ceiling. If
// aMax is your actuator's real maximum, derate it before passing it in.
// setup() prints the derived limits so the actual figure is visible.

#include <JerkPercentProfile.h>

// Stand-in for a real actuator. A real implementation would write pos out
// to hardware (servo.write(), stepper step pulses, ODrive command, etc.);
// this one just stores it so the sketch has something to print.
struct SimulatedMotor {
    float position = 0.0f;
    void moveTo(float pos) { position = pos; }
};

// 66% is the figure an MEI controller treats as the "optimum" -- a sweet
// spot rather than a mathematical one. Higher percentages smooth more but
// demand more acceleration headroom; 100% needs double the nominal aMax.
JerkPercentProfile profile(66.0f);
SimulatedMotor motor;

unsigned long moveStartMs = 0;
unsigned long lastPrintMs = 0;

void setup() {
    Serial.begin(115200);

    TrajectoryLimits limits;
    limits.vMax = 90.0f;   // units/second
    limits.aMax = 180.0f;  // units/second^2 -- NOMINAL, will be exceeded
                           // jMax is ignored: jerk comes from the percentage

    // 3-arg form -- targetDuration defaults to 0.0f (minimum time given
    // the limits above). Works directly on this concrete JerkPercentProfile,
    // not just through an ITrajectoryProfile&/*, thanks to the `using
    // ITrajectoryProfile::plan;` line in JerkPercentProfile.h.
    //
    // Phase boundaries for this move:
    //   0.000-0.165  jerk up      accel 0 -> 269
    //   0.165-0.335  hold accel   at the derived 269
    //   0.335-0.500  jerk down    accel 269 -> 0, reaching vMax
    //   0.500-1.000  cruise       at vMax, accel 0
    //   1.000-1.165  jerk down    accel 0 -> -269
    //   1.165-1.335  hold decel   at -269
    //   1.335-1.500  jerk up      accel -269 -> 0, arriving at rest
    //
    // Note the acceleration phase ends at exactly 0.500 s -- the same
    // instant the trapezoid's does -- and the move ends at exactly 1.500 s,
    // the trapezoid's duration. That equality is what the profile is for.
    profile.plan(0.0f, 90.0f, limits);

    const TrajectoryLimits& derived = profile.getDerivedLimits();
    Serial.print("jerkPercent=");   Serial.print(profile.getJerkPercent(), 1);
    Serial.print("%  nominal aMax="); Serial.print(limits.aMax, 1);
    Serial.print("  derived aMax="); Serial.print(derived.aMax, 1);
    Serial.print(" ("); Serial.print(derived.aMax / limits.aMax, 3);
    Serial.print("x)  derived jerk="); Serial.println(derived.jMax, 1);
    Serial.print("duration=");       Serial.println(profile.getDuration(), 3);

    moveStartMs = millis();
}

void loop() {
    float t = (millis() - moveStartMs) / 1000.0f;

    float pos, vel, accel;
    bool moving = profile.evaluate(t, pos, vel, accel);
    motor.moveTo(pos);

    if (millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();
        Serial.print("t=");      Serial.print(t, 3);
        Serial.print(" pos=");   Serial.print(motor.position, 2);
        Serial.print(" vel=");   Serial.print(vel, 2);
        Serial.print(" accel="); Serial.print(accel, 2);
        Serial.println(moving ? "" : "  (settled)");
    }
}
