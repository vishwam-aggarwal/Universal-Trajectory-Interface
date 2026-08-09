// SimpleTrajectoryDemo
//
// Drives one simulated motor axis through a trapezoidal motion profile.
// No real hardware is required -- SimulatedMotor just remembers the
// commanded position instead of writing it to a servo/stepper/driver, so
// this sketch compiles and runs identically on any AVR, SAMD, or Teensy
// board with nothing wired up.

#include <TrapezoidalProfile.h>

// Stand-in for a real actuator. A real implementation would write pos out
// to hardware (servo.write(), stepper step pulses, ODrive command, etc.);
// this one just stores it so the sketch has something to print.
struct SimulatedMotor {
    float position = 0.0f;
    void moveTo(float pos) { position = pos; }
};

TrapezoidalProfile profile;
SimulatedMotor motor;

unsigned long moveStartMs = 0;
unsigned long lastPrintMs = 0;

void setup() {
    Serial.begin(115200);

    TrajectoryLimits limits;
    limits.vMax = 90.0f;   // units/second
    limits.aMax = 180.0f;  // units/second^2

    // 3-arg form -- targetDuration defaults to 0.0f (minimum time given
    // the limits above). Works directly on this concrete TrapezoidalProfile,
    // not just through an ITrajectoryProfile&/*, thanks to the `using
    // ITrajectoryProfile::plan;` line in TrapezoidalProfile.h.
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
        Serial.print("t=");    Serial.print(t, 3);
        Serial.print(" pos="); Serial.print(motor.position, 2);
        Serial.print(" vel="); Serial.print(vel, 2);
        Serial.println(moving ? "" : "  (settled)");
    }
}
