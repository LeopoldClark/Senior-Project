/*

Developer:      Leopold Clark
Date Created:   September 30, 2026
Date Editied:   September 30, 2026
Editors:        Leopold Clark
Last Editor:    Leopold Clark

Intended Board: ESP32-WROOM-32, Arduino-ESP32 3.x

USB serial: 115200 baud
use xbox_bridge.py from the Xbox_Boat_Test package
close Arduino Serial Monitor before connecting the bridge

RT = proportional forward request
LT = reduce forward command to neutral, pause, then reverse
release both triggers = ramp to neutral
press both triggers = ramp to neutral
B / bridge stop = immediate neutral command and disarm
release triggers and select arm motors before starting again

commands during 3 second startup cannot arm motors
no auto startup motion
motion continues while armed and fresh commands arrive
old 2 second run limit and F/B/S commands replaced by bridge controls
missing commands for more than 300 ms = neutral and disarm
changing direction passes neutral for at least 300 ms
neutral command does not confirm the shaft or boat has stopped

ESCs assumed to support both directions with 1500 microsecond neutral
motor ESC ground and esp32 ground must be shared
remove propellers for bench testing
both ESC signal wires use GPIO 25 with the current pin settings
change rp to 26 and move that signal wire for separate outputs
125 us offset = .25 of an assumed 500 us command range (not power)
--> pulse limits, ramp rates and 300 ms pause to be trimmed

FSM and throttle ramp:
state decides operating mode
trigger position decides requested pulse offset
ramp changes the pulse gradually within the permitted direction
this is a hybrid FSM, not a strict Moore machine with fixed outputs

bridge commands: M1 rightTrigger leftTrigger enable
trigger values: 0 to 1000, enable: 0 or 1, newline ends packet
bridge telemetry: M1 uptime state pulse armed
telemetry state names remain uppercase to match the existing bridge

*/

// include statements
#include <Arduino.h>    // enables access to arduino functions and definitions
#include <math.h>       // enables rounding and float comparisons
#include <stdlib.h>     // enables text to integer conversion
#include <string.h>     // enables command prefix comparison

// constexpr prevents accidental changes to these settings
constexpr int lp =             25;     // gpio pin of left motor esc signal
constexpr int rp =             25;     // same pin feeds both esc signal wires
// motors can use separate pins later by changing rp to 26
constexpr int pwmb =           16;     // pwm resolution in bits
constexpr int pwmhz =          50;     // esc control signal frequency
constexpr int nus =          1500;     // neutral esc pulse width (us)
constexpr int fus =          1625;     // forward limit (us), matches current bridge
constexpr int bus =          1375;     // backward limit (us), matches current bridge
constexpr uint32_t startms = 3000;     // startup delay (3 seconds)
constexpr uint32_t npausems = 300;     // neutral pause before direction change
constexpr uint32_t timeoutms = 300;    // maximum gap between bridge commands
constexpr uint32_t reportms = 100;     // time between telemetry reports
constexpr float accel =     125.0f;   // pulse offset increase per second (us/s)
constexpr float decel =     250.0f;   // pulse offset decrease per second (us/s)

// enum declarations
// State: operating modes stored using int
enum class State:
    int {   startup,
            disarmed,
            stopped,
            forward,
            backward,
            neutralwait,
            fault};

// Request: requested movement direction stored using int
enum class Request:
    int {   stop,
            forward,
            backward};

// variable declarations
State state = State::startup;         // current operating state
Request request = Request::stop;      // latest requested movement
uint32_t enteredat = 0;               // time current state was entered
uint32_t startedat = 0;               // time board setup finished
uint32_t lastcommand = 0;             // time last valid command arrived
uint32_t lasttick = 0;                // time throttle was last updated
uint32_t neutralat = 0;               // time output reached neutral
uint32_t lastreport = 0;              // time last telemetry was sent
bool pwmReady = false;                // pwm setup and output tracker
bool armed = false;                   // motor motion permission
bool neutralseen = false;             // released triggers seen while disarmed
bool commandseen = false;             // at least one valid packet received
bool waiting = false;                 // direction change waiting at neutral
int rt = 0;                          // right trigger request (0 to 1000)
int lt = 0;                          // left trigger request (0 to 1000)
int lastdir = 0;                     // previous direction (-1, 0 or 1)
float offset = 0;                    // current pulse offset from neutral (us)
char input[64];                     // incoming command storage
int used = 0;                       // number of stored command characters
bool discard = false;               // ignore damaged packet until newline

// function declarations
uint32_t dutyFor(int pulseUs);       // converts pulse width to pwm duty
int pulseForState(State s);         // pulls current pulse for a state
const char* stateName(State s);     // pulls state's bridge label
void disablePWM();                 // deactivates pwm outputs
bool applyOutputs();               // applies current output to motor pins
void enterState(State next);       // changes state and records entry time
void stopMotors(uint32_t now);      // sets neutral and removes motion permission
void readBridgeCommands(uint32_t now); // collects complete serial packets
void parseCommand(uint32_t now);    // checks packet and updates requests
void checkTimeout(uint32_t now);    // disarms when commands stop arriving
void updateThrottle(uint32_t now);  // ramps pulse and handles reversal pause
int direction(float value);        // returns positive, negative or zero sign
State nextState(uint32_t now);      // next state selector
void reportStatus(uint32_t now);    // sends bridge telemetry
void setup();                      // board initializer       (arduino standard)
void loop();                       // main control function   (arduino standard)

// function definitions
// board initializer   (arduino standard)
void setup() {

    Serial.begin(115200);           // initialize USB serial communication
    pinMode(lp, OUTPUT);            // initialize left motor signal pin
    digitalWrite(lp, LOW);          // initialize signal low

    if (rp != lp) {                 // only configure right pin if separate

        pinMode(rp, OUTPUT);
        digitalWrite(rp, LOW);

    }

    const bool lok = ledcAttach(lp, pwmhz, pwmb);
    bool rok = lok;                 // shared signal uses the same pwm output

    if (rp != lp) {

        rok = ledcAttach(rp, pwmhz, pwmb);

    }

    pwmReady = lok && rok;          // pwm ready only if both outputs configured
    startedat = millis();
    lasttick = startedat;
    neutralat = startedat;

    if (pwmReady) {

        enterState(State::startup);

    } else {

        enterState(State::fault);

    }

    if (!applyOutputs()) {

        enterState(State::fault);

    }

}

// main control function   (arduino standard)
void loop() {

    const uint32_t now = millis();

    checkTimeout(now);              // check old command age before reading new data
    readBridgeCommands(now);        // receive trigger and enable values
    updateThrottle(now);            // update pulse without blocking input checks

    const State next = nextState(now);

    if (next != state) {

        enterState(next);

    }

    if (!applyOutputs()) {          // pulse may change even within the same state

        enterState(State::fault);

    }

    reportStatus(now);
    delay(1);

}

uint32_t dutyFor(int pulseUs) {

    return uint32_t((uint64_t(pulseUs) * pwmhz * (1UL << pwmb)
                    + 500000ULL) / 1000000ULL);

}

int pulseForState(State s) {

    switch (s) {

        case State::forward:
        case State::backward:
            return nus + int(lroundf(offset));

        case State::startup:
        case State::disarmed:
        case State::stopped:
        case State::neutralwait:
            return nus;

        case State::fault:
            return 0;

    }

    return 0;

}

const char* stateName(State s) {

    switch (s) {

        case State::startup:     return "STARTUP";
        case State::disarmed:    return "DISARMED";
        case State::stopped:     return "STOPPED";
        case State::forward:     return "FORWARD";
        case State::backward:    return "BACKWARD";
        case State::neutralwait: return "NEUTRAL_WAIT";
        case State::fault:       return "FAULT";

    }

    return "FAULT";

}

void disablePWM() {

    ledcDetach(lp);
    pinMode(lp, OUTPUT);
    digitalWrite(lp, LOW);

    if (rp != lp) {

        ledcDetach(rp);
        pinMode(rp, OUTPUT);
        digitalWrite(rp, LOW);

    }

    pwmReady = false;               // actual esc signal-loss response needs testing

}

bool applyOutputs() {

    if (state == State::fault) {

        return true;               // fault entry already disabled outputs

    }

    if (!pwmReady) {

        return false;

    }

    const uint32_t duty = dutyFor(pulseForState(state));
    const bool lok = ledcWrite(lp, duty);
    bool rok = lok;

    if (rp != lp) {

        rok = ledcWrite(rp, duty);

    }

    return lok && rok;

}

void enterState(State next) {

    state = next;
    enteredat = millis();

    if (state == State::fault) {

        stopMotors(enteredat);
        disablePWM();

    }

}

void stopMotors(uint32_t now) {

    if (offset != 0) {

        neutralat = now;            // start neutral dwell after motion command ends

    }

    offset = 0;
    rt = 0;
    lt = 0;
    armed = false;
    neutralseen = false;
    waiting = false;
    request = Request::stop;

}

void checkTimeout(uint32_t now) {

    if (!commandseen || uint32_t(now - lastcommand) > timeoutms) {

        stopMotors(now);

    }

}

void readBridgeCommands(uint32_t now) {

    for (int count = 0; count < 128 && Serial.available(); ++count) {

        const char c = char(Serial.read());

        if (c == '\n') {

            if (!discard && used > 0) {

                input[used] = '\0';
                parseCommand(now);

            }

            used = 0;
            discard = false;

        } else if (!discard) {

            if (c < ' ' || c > '~' || used >= int(sizeof(input)) - 1) {

                discard = true;
                used = 0;
                stopMotors(now);    // reject damaged or oversized command

            } else {

                input[used++] = c;

            }

        }

    }

}

void parseCommand(uint32_t now) {

    if (strncmp(input, "M1 ", 3) != 0) {

        stopMotors(now);
        return;

    }

    char* p = input + 3;
    long values[3];                 // right trigger, left trigger, enable

    for (int i = 0; i < 3; ++i) {

        char* begin = p;
        int digits = 0;

        while (*p >= '0' && *p <= '9') {

            ++p;
            ++digits;

        }

        if (digits < 1 || digits > 4) {

            stopMotors(now);
            return;

        }

        values[i] = strtol(begin, nullptr, 10);

        if (i < 2) {

            if (*p != ' ') {

                stopMotors(now);
                return;

            }

            ++p;

        } else if (*p != '\0') {

            stopMotors(now);
            return;

        }

    }

    if (values[0] > 1000 || values[1] > 1000 || values[2] > 1) {

        stopMotors(now);
        return;

    }

    lastcommand = now;
    commandseen = true;

    if (uint32_t(now - startedat) < startms || state == State::fault || !pwmReady) {

        stopMotors(now);
        return;

    }

    if (values[2] == 0) {           // bridge stop or disarmed command

        stopMotors(now);
        neutralseen = values[0] == 0 && values[1] == 0;
        return;

    }

    if (!armed) {

        if (!neutralseen || values[0] != 0 || values[1] != 0) {

            return;                // require released triggers before arming

        }

        armed = true;
        neutralseen = false;

    }

    rt = int(values[0]);
    lt = int(values[1]);
    request = Request::stop;

    if (rt > 0 && lt == 0) {

        request = Request::forward;

    } else if (lt > 0 && rt == 0) {

        request = Request::backward;

    }

}

int direction(float value) {

    if (value > 0) {

        return 1;

    }

    if (value < 0) {

        return -1;

    }

    return 0;

}

void updateThrottle(uint32_t now) {

    uint32_t elapsed = uint32_t(now - lasttick);
    lasttick = now;
    waiting = false;

    if (!armed || state == State::fault || uint32_t(now - startedat) < startms) {

        offset = 0;
        return;

    }

    if (elapsed > 50) {

        elapsed = 50;              // limit ramp jump if a loop iteration is late

    }

    const float seconds = elapsed / 1000.0f;
    float target = 0;

    if (request == Request::forward) {

        target = (fus - nus) * (rt / 1000.0f);

    } else if (request == Request::backward) {

        target = (bus - nus) * (lt / 1000.0f);

    }

    const int wanted = direction(target);

    if (offset != 0 && wanted != 0 && wanted != direction(offset)) {

        target = 0;                // reach neutral before applying opposite command

    }

    if (offset == 0 && wanted != 0 && lastdir != 0 && wanted != lastdir) {

        if (uint32_t(now - neutralat) < npausems) {

            waiting = true;
            return;

        }

    }

    float rate = decel;

    if (fabsf(target) > fabsf(offset)) {

        rate = accel;

    }

    const float before = offset;
    const float step = rate * seconds;

    if (offset < target) {

        offset = fminf(offset + step, target);

    } else {

        offset = fmaxf(offset - step, target);

    }

    if (offset != 0) {

        lastdir = direction(offset);

    }

    if (before != 0 && offset == 0) {

        neutralat = now;            // start pause when ramp actually reaches zero

    }

}

State nextState(uint32_t now) {

    if (state == State::fault || !pwmReady) {

        return State::fault;

    }

    if (uint32_t(now - startedat) < startms) {

        return State::startup;

    }

    if (!armed) {

        return State::disarmed;

    }

    if (waiting) {

        return State::neutralwait;

    }

    if (offset > 0) {

        return State::forward;

    }

    if (offset < 0) {

        return State::backward;

    }

    return State::stopped;

}

void reportStatus(uint32_t now) {

    if (uint32_t(now - lastreport) < reportms) {

        return;

    }

    lastreport = now;
    char report[96];
    const int n = snprintf(report, sizeof(report), "M1 %lu %s %d %d\n",
        (unsigned long)now, stateName(state), pulseForState(state), int(armed));

    if (n > 0 && n < int(sizeof(report)) && Serial.availableForWrite() >= n) {

        Serial.write((const uint8_t*)report, n);

    }

}
