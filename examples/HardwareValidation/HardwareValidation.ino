// HardwareValidation
//
// Runs the same physical move under all three profiles and streams both the
// commanded and the MEASURED angle over serial, so the profile shapes can be
// checked against real hardware instead of only against a desktop build.
//
// Unlike the other examples in this folder, this one needs hardware:
//
//   - a 180 deg hobby servo, signal on pin A3
//   - an AS5600 magnetic encoder on I2C (A4/SDA, A5/SCL) reading the angle
//     of the servo horn, with its magnet centred on the output shaft
//   - the servo powered from its OWN 5V supply, not the board's regulator --
//     a servo under load will brown out a USB-powered board and reset it
//     mid-capture. Tie the supply ground to the board ground.
//
// Secure the horn before running: it sweeps 90 deg, three times, without
// warning.
//
// Protocol (kept deliberately dumb so tools/capture_validation.py can drive
// it): the sketch prints "# READY" and waits. Send 'g' and it emits
// "# BEGIN", a CSV header, the rows, then "# END". Anything starting with
// '#' is a comment line, not data.

#include <Servo.h>
#include <Wire.h>
#include <TrapezoidalProfile.h>
#include <SCurveProfile.h>
#include <JerkPercentProfile.h>

// ---------------------------------------------------------------- config --
static const int   SERVO_PIN     = A3;

// ---- servo pulse-width mapping: MEASURED, not assumed -------------------
// "1000-2000us spans 180 deg" is a convention, not a specification, and
// getting it wrong does not look like a calibration error -- it looks like a
// profile the servo is failing to track, because the measured curve has the
// right SHAPE at the wrong AMPLITUDE. Run the 'c' calibration mode on your
// own servo and put its numbers here.
//
// Measured on the servo this example was developed against (send 'c', fit a
// line through the result): 0.0888 deg/us, i.e. 88.75 deg across the whole
// 1000-2000us range -- so it is a ~90 deg servo on standard pulse widths, not
// a 180 deg one. Linear to within 1.5 deg worst case (0.52 deg RMS) with
// 0.45 deg of hysteresis between the up and down passes.
static const int   SERVO_MIN_US  = 1000;    // universally safe hobby-servo range;
static const int   SERVO_MAX_US  = 2000;    // going wider risks a hard stop
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 88.75f;  // <-- measured span, not 180

// The move: 80 deg, sitting a few degrees inside each end of the measured
// travel so neither end runs into a stop. Smaller than the 90 deg the desktop
// demos use, because that is what this servo physically has.
static const float Q0 = 4.0f;
static const float QF = 84.0f;

// Same limits as the other three demos, so the durations are the familiar
// 1.50 / 1.75 / 1.50 s and the numbers line up with the desktop tests.
// Same limits as the other three demos. Deliberately gentle: at these
// numbers the servo actually tracks the profile, which is what makes the
// comparison meaningful. Raise them far enough (400 / 4000 / 20000 was tried
// during development) and the servo simply saturates -- it stops tracking
// any of the three and the shapes no longer matter.
static const float V_MAX        = 90.0f;    // deg/s
static const float A_MAX        = 180.0f;   // deg/s^2  (NOMINAL for jerk-percent)
static const float J_MAX        = 720.0f;   // deg/s^3  (SCurveProfile only)
static const float JERK_PERCENT = 66.0f;    // JerkPercentProfile only

static const unsigned long SAMPLE_US   = 5000UL;    // 200 Hz
static const unsigned long SETTLE_MS   = 700UL;     // keep sampling after the
                                                    // move ends -- overshoot
                                                    // and ringing live here
static const unsigned long HOME_MS     = 1200UL;    // let the horn stop moving
                                                    // before zeroing the encoder

// ------------------------------------------------------------- AS5600 I2C --
static const uint8_t AS5600_ADDR      = 0x36;
static const uint8_t AS5600_RAW_ANGLE = 0x0C;  // unfiltered 12-bit angle
static const float   COUNTS_PER_REV   = 4096.0f;

// Reads the raw 12-bit angle. Returns false if the encoder did not respond,
// which is the difference between "the horn did not move" and "the wiring is
// wrong" -- worth being able to tell apart.
static bool as5600Read(uint16_t& counts) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) return false;
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    counts = (((uint16_t)hi << 8) | lo) & 0x0FFF;
    return true;
}

// Unwrapped angle accumulator. The magnet is absolute over one turn, so a
// move that happens to straddle the 0/4095 seam would otherwise show as a
// 360 deg jump; track deltas and take the short way round instead.
static long     s_accum   = 0;      // accumulated counts since zeroing
static uint16_t s_lastRaw = 0;
static bool     s_haveRaw = false;

static void encoderZero() {
    uint16_t raw;
    if (as5600Read(raw)) { s_lastRaw = raw; s_haveRaw = true; }
    s_accum = 0;
}

static bool encoderReadDeg(float& deg) {
    uint16_t raw;
    if (!as5600Read(raw)) return false;
    if (!s_haveRaw) { s_lastRaw = raw; s_haveRaw = true; }
    int delta = (int)raw - (int)s_lastRaw;
    if (delta >  2048) delta -= 4096;        // wrapped backwards
    if (delta < -2048) delta += 4096;        // wrapped forwards
    s_accum += delta;
    s_lastRaw = raw;
    deg = (float)s_accum * (360.0f / COUNTS_PER_REV);
    return true;
}

// -------------------------------------------------------------- profiles --
Servo servo;

TrapezoidalProfile trapezoidal;
SCurveProfile      scurve;
JerkPercentProfile jerkPercent(JERK_PERCENT);

ITrajectoryProfile* profiles[3] = { &trapezoidal, &scurve, &jerkPercent };
const char*         names[3]    = { "trap", "scurve", "jerkpct" };

// Commands an angle using writeMicroseconds, NOT write(). write() takes whole
// degrees, so it would quantise a smooth profile into 1 deg stair steps --
// injecting exactly the kind of discontinuity this sketch exists to measure
// the absence of. Microseconds keep the command as smooth as the profile.
static void commandDeg(float deg) {
    if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
    if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;
    const float frac = (deg - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
    servo.writeMicroseconds((int)(SERVO_MIN_US + frac * (SERVO_MAX_US - SERVO_MIN_US) + 0.5f));
}

static bool planOne(int i) {
    TrajectoryLimits lim;
    lim.vMax = V_MAX;
    lim.aMax = A_MAX;
    // jMax is used by SCurveProfile, ignored by the other two.
    lim.jMax = (profiles[i] == &scurve) ? J_MAX : 0.0f;
    return profiles[i]->plan(Q0, QF, lim, 0.0f);
}

// Runs one profile start to finish, printing one CSV row per sample.
static void runOne(int i) {
    // Park at the start pose and let the horn actually get there and stop --
    // zeroing the encoder while it is still moving would bias every sample.
    commandDeg(Q0);
    delay(HOME_MS);
    encoderZero();

    if (!planOne(i)) {
        Serial.print(F("# ERROR plan failed for "));
        Serial.println(names[i]);
        return;
    }

    const float duration = profiles[i]->getDuration();
    const unsigned long totalUs =
        (unsigned long)(duration * 1000000.0f) + SETTLE_MS * 1000UL;

    const unsigned long t0 = micros();
    unsigned long next = t0;

    for (;;) {
        const unsigned long now = micros();
        if ((long)(now - next) < 0) continue;      // busy-wait to the next slot
        next += SAMPLE_US;

        const unsigned long elapsedUs = now - t0;
        if (elapsedUs > totalUs) break;

        const float t = elapsedUs / 1000000.0f;

        float pos, vel, accel;
        profiles[i]->evaluate(t, pos, vel, accel);
        commandDeg(pos);

        float measRel;
        const bool ok = encoderReadDeg(measRel);

        // Measured is reported relative to the start pose, then offset by Q0
        // so it shares an axis with the commanded angle. Sign is NOT corrected
        // here -- the encoder's direction relative to the servo's depends on
        // how the magnet is oriented, and the plotting script detects it.
        Serial.print(names[i]);          Serial.print(',');
        Serial.print(elapsedUs / 1000);  Serial.print(',');
        Serial.print(pos, 3);            Serial.print(',');
        if (ok) Serial.print(Q0 + measRel, 3); else Serial.print(F("nan"));
        Serial.print(',');
        Serial.println(vel, 3);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }               // needed on native-USB boards
    Wire.begin();
    Wire.setClock(400000);              // 2 byte read comfortably inside 10 ms
    servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    commandDeg(Q0);

    uint16_t probe;
    Serial.println(F("# HardwareValidation"));
    Serial.print(F("# encoder: "));
    Serial.println(as5600Read(probe) ? F("AS5600 responding") : F("NOT RESPONDING - check wiring"));
    Serial.print(F("# move: "));  Serial.print(Q0);      Serial.print(F(" -> "));
    Serial.print(QF);             Serial.print(F(" deg, vMax ")); Serial.print(V_MAX);
    Serial.print(F(", aMax "));   Serial.print(A_MAX);
    Serial.print(F(", jMax "));   Serial.print(J_MAX);
    Serial.print(F(", jerkPercent ")); Serial.println(JERK_PERCENT);
    Serial.println(F("# send 'g' to run the validation, 'c' to calibrate the servo"));
    Serial.println(F("# READY"));
}

// Sweeps the pulse width across a conservative span, dwelling at each step so
// the horn actually arrives, and reports what the encoder read. Fitting a line
// through the result gives degrees-per-microsecond for THIS servo.
//
// This exists because the degrees->microseconds mapping is the one number in
// this sketch that cannot be known a priori: "1000-2000us over 0-180 deg" is a
// convention, not a specification, and a servo that actually wants 500-2500us
// will travel roughly half as far as commanded -- which looks exactly like a
// profile that is not being tracked, and is not.
static void runCalibration() {
    const int CAL_MIN_US = 1000;   // 1000-2000us is the universally safe
    const int CAL_MAX_US = 2000;   // hobby-servo range; wider risks a hard stop
    const int CAL_STEP   = 50;

    Serial.println(F("# CAL BEGIN"));
    Serial.println(F("dir,us,meas_deg"));

    commandDeg(Q0);
    servo.writeMicroseconds(CAL_MIN_US);
    delay(1200);
    encoderZero();

    float deg;
    for (int us = CAL_MIN_US; us <= CAL_MAX_US; us += CAL_STEP) {
        servo.writeMicroseconds(us);
        delay(350);
        if (encoderReadDeg(deg)) {
            Serial.print(F("up,")); Serial.print(us); Serial.print(',');
            Serial.println(deg, 4);
        }
    }
    // Back down as well: the gap between the two passes is backlash plus the
    // servo's own deadband, and it belongs in the record rather than averaged
    // away silently.
    for (int us = CAL_MAX_US; us >= CAL_MIN_US; us -= CAL_STEP) {
        servo.writeMicroseconds(us);
        delay(350);
        if (encoderReadDeg(deg)) {
            Serial.print(F("down,")); Serial.print(us); Serial.print(',');
            Serial.println(deg, 4);
        }
    }

    commandDeg(Q0);
    Serial.println(F("# CAL END"));
}

void loop() {
    const int c = Serial.read();
    if (c == 'c') { runCalibration(); return; }
    if (c != 'g') return;

    Serial.println(F("# BEGIN"));
    for (int i = 0; i < 3; ++i) {
        if (planOne(i)) {
            Serial.print(F("# profile ")); Serial.print(names[i]);
            Serial.print(F(" duration "));  Serial.println(profiles[i]->getDuration(), 4);
        }
    }
    // The jerk-percent axis exceeds the nominal aMax on purpose; report what
    // it actually derived so the capture records it rather than assuming.
    Serial.print(F("# jerkpct derived aMax "));
    Serial.print(jerkPercent.getDerivedLimits().aMax, 3);
    Serial.print(F(" jerk "));
    Serial.println(jerkPercent.getDerivedLimits().jMax, 3);

    Serial.println(F("profile,t_ms,cmd_deg,meas_deg,cmd_vel"));
    for (int i = 0; i < 3; ++i) runOne(i);

    commandDeg(Q0);
    Serial.println(F("# END"));
}
