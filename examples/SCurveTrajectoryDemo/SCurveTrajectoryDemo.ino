// SCurveTrajectoryDemo
//
// Drives one simulated motor axis through an S-curve (jerk-limited) motion
// profile. Same structure as SimpleTrajectoryDemo, with SCurveProfile in
// place of TrapezoidalProfile -- no real hardware is required, since
// SimulatedMotor just remembers the commanded position instead of writing
// it to a servo/stepper/driver. Compiles and runs identically on any AVR,
// SAMD, or Teensy board with nothing wired up.
//
// The difference from the trapezoidal demo is what happens to acceleration.
// A trapezoidal profile steps accel instantly from 0 to aMax; this one ramps
// it at no more than jMax, so the accel column climbs and falls instead of
// jumping. That smoothness costs time: the same 90-unit move takes 1.75 s
// here versus 1.50 s trapezoidal.

#include <SCurveProfile.h>

// Stand-in for a real actuator. A real implementation would write pos out
// to hardware (servo.write(), stepper step pulses, ODrive command, etc.);
// this one just stores it so the sketch has something to print.
struct SimulatedMotor {
    float position = 0.0f;
    void moveTo(float pos) { position = pos; }
};

SCurveProfile profile;
SimulatedMotor motor;

unsigned long moveStartMs = 0;
unsigned long lastPrintMs = 0;

void setup() {
    Serial.begin(115200);

    TrajectoryLimits limits;
    limits.vMax = 90.0f;   // units/second
    limits.aMax = 180.0f;  // units/second^2
    limits.jMax = 720.0f;  // units/second^3 -- 0 would be rejected by
                           // SCurveProfile (it means "trapezoidal" in
                           // TrajectoryLimits, not "no jerk limit")

    // 3-arg form -- targetDuration defaults to 0.0f (minimum time given
    // the limits above). Works directly on this concrete SCurveProfile,
    // not just through an ITrajectoryProfile&/*, thanks to the `using
    // ITrajectoryProfile::plan;` line in SCurveProfile.h.
    //
    // With these limits the move uses all seven segments, and the phase
    // boundaries land on exact quarter-seconds:
    //   0.00-0.25  jerk up      accel 0 -> 180
    //   0.25-0.50  hold accel   at aMax
    //   0.50-0.75  jerk down    accel 180 -> 0, reaching vMax
    //   0.75-1.00  cruise       at vMax, accel 0
    //   1.00-1.25  jerk down    accel 0 -> -180
    //   1.25-1.50  hold decel   at -aMax
    //   1.50-1.75  jerk up      accel -180 -> 0, arriving at rest
    profile.plan(0.0f, 90.0f, limits);
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
