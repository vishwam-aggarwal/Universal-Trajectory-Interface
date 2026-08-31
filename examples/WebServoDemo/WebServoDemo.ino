// WebServoDemo
//
// Firmware for the Trajectory Lab web app's "Connect hardware" tab
// (https://vishwamaggarwal.com/tools/trajectory-lab/). The page flashes
// this sketch over Web Serial, then commands moves and plots what the
// board streams back against the same profile computed in the browser.
//
// One Arduino, one hobby servo, nothing else. Deliberately NOT
// examples/HardwareValidation, which hard-requires an AS5600 magnetic
// encoder on I2C -- precisely the part a visitor will not have -- and
// which only runs one canned sequence rather than taking commands.
//
// ---------------------------------------------------------------------
// Everything here is in MICROSECONDS of pulse width, not degrees
// ---------------------------------------------------------------------
// Without an encoder there is no way to know what a pulse width does to
// this particular servo, and the usual assumption is wrong. The servo on
// the bench rig for this library moves 0.0888 deg/us, so the standard
// 1000-2000 us range spans 89.7 deg -- not the 180 deg nearly every
// tutorial assumes. Commanding as if it were 180 produces a curve with
// the right shape at the wrong amplitude, which reads like the servo
// failing to track rather than like a calibration error, and that cost
// real time to diagnose once already.
//
// So this sketch commands and reports pulse width, which is the one
// quantity it actually knows. The web page lets you enter your own
// measured deg/us if you want a second axis, and labels it as your
// figure rather than as a measurement.
//
// ---------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------
//   Servo signal  -> pin A3
//   Servo V+      -> its OWN 4.8-6 V supply, NOT the board's 5 V pin
//   Servo ground  -> that supply's ground AND the board's ground
//
// A servo's stall current will brown out a USB-powered board mid-move,
// which looks like the profile glitching. Give it its own supply.
//
// ---------------------------------------------------------------------
// Serial protocol -- 115200 baud, one command per line, "\n"-terminated
// ---------------------------------------------------------------------
//   PING                          -> OK PING
//   ID                            -> OK ID UTI-WebServoDemo 1
//   TYPE trap|scurve|jerkpct      -> OK TYPE      | ERR USAGE
//   LIM <vMax> <aMax> <jMax> <jp> -> OK LIM       | ERR USAGE
//   RANGE <minUs> <maxUs>         -> OK RANGE     | ERR RANGE
//   MOVE <targetUs>               -> OK MOVE      | ERR RANGE | ERR PLAN_FAILED
//   HOME                          -> OK HOME      | ERR PLAN_FAILED
//   STOP                          -> OK STOP      | ERR ALREADY_IDLE
//
// Any command other than PING and STOP is refused with ERR BUSY while a
// move is running. STOP is exempt on purpose: it is the one thing that
// must always get through.
//
// MOVE and HOME acknowledge immediately, then stream until the move
// settles:
//   TELEM <ms> <pos_x100> <vel_x100> <accel_x100>
//   <ms> DONE <pos_x100>
//
// Values are hundredths, as integers, because Serial.print(float) on AVR
// pulls in ~1.5 KB of formatting code and is slow enough to disturb a
// 100 Hz loop. Lines beginning with '#' are comments, never data.

#include <Servo.h>
#include <TrapezoidalProfile.h>
#include <SCurveProfile.h>
#include <JerkPercentProfile.h>

// ------------------------------------------------------------- config --

static const int SERVO_PIN = A3;

// Hard bounds. attach() clamps writeMicroseconds() to these, so they are
// the real backstop against a command driving the horn into a mechanical
// stop; the RANGE command moves the soft limits inside them.
static const int ABS_MIN_US = 800;
static const int ABS_MAX_US = 2200;

static const int DEFAULT_MIN_US = 1000;   // the conventional safe span
static const int DEFAULT_MAX_US = 2000;
static const int HOME_US        = 1500;

// Defaults chosen so the durations match the three desktop example
// sketches exactly -- 1.50 s trapezoidal, 1.75 s S-curve, 1.50 s
// jerk-percent -- which makes the board's numbers directly comparable to
// the simulator's. They are the desktop limits scaled by the ratio of a
// 1000 us span to a 90 unit one.
static const float DEFAULT_V_MAX = 1000.0f;   // us/s
static const float DEFAULT_A_MAX = 2000.0f;   // us/s^2
static const float DEFAULT_J_MAX = 8000.0f;   // us/s^3
static const float DEFAULT_JERK_PERCENT = 66.0f;

static const unsigned long SAMPLE_US = 10000UL;  // 100 Hz
static const unsigned long SETTLE_MS = 300UL;    // keep streaming past the
                                                 // end of the move, so the
                                                 // settle is visible

static const uint8_t LINE_MAX = 48;

// -------------------------------------------------------------- state --

Servo servo;

TrapezoidalProfile trapProfile;
SCurveProfile      scurveProfile;
JerkPercentProfile jerkPctProfile(DEFAULT_JERK_PERCENT);

enum ProfileKind { KIND_TRAP = 0, KIND_SCURVE = 1, KIND_JERKPCT = 2 };
ProfileKind kind = KIND_TRAP;

TrajectoryLimits limits(DEFAULT_V_MAX, DEFAULT_A_MAX, DEFAULT_J_MAX);
float jerkPercent = DEFAULT_JERK_PERCENT;

int minUs = DEFAULT_MIN_US;
int maxUs = DEFAULT_MAX_US;

float currentUs = HOME_US;      // last commanded pulse width

bool          moving = false;
unsigned long moveStartMs = 0;
unsigned long lastSampleUs = 0;
float         moveDuration = 0.0f;

char    line[LINE_MAX];
uint8_t lineLen = 0;

// ----------------------------------------------------------- helpers --

ITrajectoryProfile* activeProfile() {
    switch (kind) {
        case KIND_SCURVE:  return &scurveProfile;
        case KIND_JERKPCT: return &jerkPctProfile;
        default:           return &trapProfile;
    }
}

void commandUs(float us) {
    if (us < ABS_MIN_US) us = ABS_MIN_US;
    if (us > ABS_MAX_US) us = ABS_MAX_US;
    currentUs = us;
    // writeMicroseconds(), never write(): write() takes whole degrees and
    // would quantise the profile into 1-degree stair steps, injecting
    // exactly the discontinuity this demo exists to show the absence of.
    servo.writeMicroseconds((int)(us + 0.5f));
}

// Hundredths as an integer -- see the note on float printing above.
long centi(float v) { return (long)(v * 100.0f + (v >= 0 ? 0.5f : -0.5f)); }

void emitTelem(unsigned long ms, float pos, float vel, float accel) {
    Serial.print(F("TELEM "));
    Serial.print(ms);          Serial.print(' ');
    Serial.print(centi(pos));  Serial.print(' ');
    Serial.print(centi(vel));  Serial.print(' ');
    Serial.println(centi(accel));
}

bool startMove(float targetUs) {
    if (targetUs < minUs || targetUs > maxUs) {
        Serial.println(F("ERR RANGE"));
        return false;
    }

    if (kind == KIND_JERKPCT) {
        jerkPctProfile.setJerkPercent(jerkPercent);
    }

    if (!activeProfile()->plan(currentUs, targetUs, limits, 0.0f)) {
        // Reachable for a non-positive limit, for jMax == 0 on an S-curve
        // (rejected rather than quietly downgraded to a trapezoid), and
        // for a jerk percentage outside (0, 100].
        Serial.println(F("ERR PLAN_FAILED"));
        return false;
    }

    moveDuration = activeProfile()->getDuration();
    moveStartMs  = millis();
    lastSampleUs = micros();
    moving       = true;
    Serial.println(F("OK MOVE"));
    return true;
}

// ---------------------------------------------------------- commands --

// Returns the next whitespace-delimited token, or NULL. strtok state is
// carried between calls, same as the C idiom.
char* nextToken() { return strtok(NULL, " \t"); }

bool tokenFloat(float& out) {
    char* t = nextToken();
    if (!t) return false;
    out = atof(t);
    return true;
}

void handleCommand(char* cmd) {
    char* verb = strtok(cmd, " \t");
    if (!verb) return;

    // PING and STOP are the two commands that must work at any time; every
    // other one is refused while a move is in flight rather than silently
    // replanning underneath it.
    if (!strcmp(verb, "PING")) { Serial.println(F("OK PING")); return; }

    if (!strcmp(verb, "STOP")) {
        if (!moving) { Serial.println(F("ERR ALREADY_IDLE")); return; }
        moving = false;
        commandUs(currentUs);          // hold wherever the profile had got to
        Serial.println(F("OK STOP"));
        Serial.print(millis() - moveStartMs);
        Serial.print(F(" DONE "));
        Serial.println(centi(currentUs));
        return;
    }

    if (moving) { Serial.println(F("ERR BUSY")); return; }

    if (!strcmp(verb, "ID")) {
        Serial.println(F("OK ID UTI-WebServoDemo 1"));
        return;
    }

    if (!strcmp(verb, "TYPE")) {
        char* t = nextToken();
        if (!t) { Serial.println(F("ERR USAGE")); return; }
        if      (!strcmp(t, "trap"))    kind = KIND_TRAP;
        else if (!strcmp(t, "scurve"))  kind = KIND_SCURVE;
        else if (!strcmp(t, "jerkpct")) kind = KIND_JERKPCT;
        else { Serial.println(F("ERR USAGE")); return; }
        Serial.println(F("OK TYPE"));
        return;
    }

    if (!strcmp(verb, "LIM")) {
        float v, a, j, p;
        if (!tokenFloat(v) || !tokenFloat(a) || !tokenFloat(j) || !tokenFloat(p)) {
            Serial.println(F("ERR USAGE"));
            return;
        }
        // Not validated here: each profile's plan() already rejects what
        // it cannot use, and reports which through ERR PLAN_FAILED. Doing
        // it twice would only risk the two disagreeing.
        limits.vMax = v;
        limits.aMax = a;
        limits.jMax = j;
        jerkPercent = p;
        Serial.println(F("OK LIM"));
        return;
    }

    if (!strcmp(verb, "RANGE")) {
        float lo, hi;
        if (!tokenFloat(lo) || !tokenFloat(hi)) { Serial.println(F("ERR USAGE")); return; }
        if (lo < ABS_MIN_US || hi > ABS_MAX_US || lo >= hi) {
            Serial.println(F("ERR RANGE"));
            return;
        }
        minUs = (int)lo;
        maxUs = (int)hi;
        Serial.println(F("OK RANGE"));
        return;
    }

    if (!strcmp(verb, "MOVE")) {
        float target;
        if (!tokenFloat(target)) { Serial.println(F("ERR USAGE")); return; }
        startMove(target);
        return;
    }

    if (!strcmp(verb, "HOME")) {
        if (startMove(HOME_US)) {
            // startMove already printed OK MOVE; say which it was.
            Serial.println(F("# homing"));
        }
        return;
    }

    Serial.println(F("ERR UNKNOWN"));
}

// -------------------------------------------------------------- setup --

void setup() {
    Serial.begin(115200);

    servo.attach(SERVO_PIN, ABS_MIN_US, ABS_MAX_US);
    commandUs(HOME_US);

    Serial.println(F("# WebServoDemo"));
    Serial.println(F("# units are microseconds of pulse width, not degrees"));
    Serial.print(F("# range "));   Serial.print(minUs);
    Serial.print(F(" - "));        Serial.print(maxUs);
    Serial.print(F(" us, home ")); Serial.println(HOME_US);
    Serial.println(F("# READY"));
}

void loop() {
    // Commands first, so STOP is seen as early as possible.
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                line[lineLen] = '\0';
                handleCommand(line);
                lineLen = 0;
            }
        } else if (lineLen < LINE_MAX - 1) {
            line[lineLen++] = c;
        } else {
            // Overlong line: drop it rather than half-executing it. The
            // trailing fragment is discarded with the newline above.
            lineLen = 0;
        }
    }

    if (!moving) return;

    unsigned long now = micros();
    if (now - lastSampleUs < SAMPLE_US) return;
    lastSampleUs += SAMPLE_US;

    unsigned long elapsedMs = millis() - moveStartMs;
    float t = elapsedMs / 1000.0f;

    float pos, vel, accel;
    activeProfile()->evaluate(t, pos, vel, accel);
    commandUs(pos);
    emitTelem(elapsedMs, pos, vel, accel);

    // Keep streaming past the end so the settle is visible, then stop.
    if (t >= moveDuration + SETTLE_MS / 1000.0f) {
        moving = false;
        Serial.print(elapsedMs);
        Serial.print(F(" DONE "));
        Serial.println(centi(currentUs));
    }
}
