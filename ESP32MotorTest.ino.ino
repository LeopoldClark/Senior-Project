/*

Developer:      Leopold Clark
Date Created:   September 29, 2026
Date Editied:   September 30, 2026
Editors:        Leopold Clark
Last Editor:    Leopold Clark

Intedned Board: ESP32-WROOM-32, Arduino-ESP32

Serial Monitor: 115200 baud
F = forward
B = backward
S = stop

Commands during 3 second startup to be ignored
no auto startup motion
motion state runs max 2 seconds
motion halts and seeks new command
repeating f/b doesn't extend running brust
direction change passess stop for >= 300 miliseconds
S cancles command
ESCs assumed to support dual direction with 1500 microsecond neutral
Motor ESC ground and esp32 ground must be shared
125 us offset = .25 of command range (not power)
--> 300 ms pause to be trimmed

Moore:
motor pulse width relies only on current state
commands and timers parameters of next state but never set motor pulses
finite run duration is for testing limitations

*/

// include statements
#include <Arduino.h>    // enables access to arduino fncs and defs

// constexpr safety net preventing program overwrite
constexpr int lp =          25;     // gpio pin of left motor esc signal
constexpr int rp =          25;     // gpio pin of right motor esc signal
// motors pin to be seperate during directional implementation
constexpr int pwmb =        16;     // pwm resolution in bits (16 yileds adequite control)
constexpr int pwmhz =      50;     // pwm frequency (50 Hz is esc standard)
constexpr int nus =             1500;   // neutral esc pulse width (ms)  - to be trimmed
constexpr int fus =             1650;   // forward esc pulse width (ms)  - to be trimmed
constexpr int bus =             1375;   // backward esc pulse width (ms) - to be trimmed
constexpr uint32_t startms =    3000;   // start up delay (= 3 seconds)
constexpr uint32_t npausems =   300;    // delay before direction swich (.3 seconds)
constexpr uint32_t runms =      2000;   // maximum duration of a trial - adjustable

uint32_t enteredAt = 0;     // current state duration storage - updates with millis()
bool pwmReady = false;      // safty pin, pwm proper set up tracker

// class declarations
// State: five states storing each as an 8bit integer
enum class State:
    int {   startup,
                stopped,
                forward,
                backward,
                fault};

// Request: three possible movement commands stored as 8bit integers
enum class Request:
    int {   stop,
                forward,
                backward};

// function delarations
uint32_t dutyFor(int pulseUs);  // convert puylse width to pwm duty cycle
int pulseForState(State s);     // pulls puse width for a state
const char* stateName(State s); // pulls state's name
void disablePWM();              // deactivates pwm outputs
bool applyOutputs();            // applies current state's output to motor pins
void enterState(State next);    // changes states and applies output
void readTestCommands();        // serail command reader
State state = State::startup;
Request request = Request::stop;
State nextState(uint32_t now);  // next state selector
void setup();                   // board initializer        (arduino standard)
void loop();                    // main control function    (arduino standard)

// function definitions
// board initializeer   (arduino standard)
void setup() {

    Serial.begin(115200);   // initialize arduino ide serial communication at 115200 baud
    // initialize motor gpio pins as outputs=
    pinMode(lp, OUTPUT);    // left motor 
    pinMode(rp, OUTPUT);    // right motor 
    // initialize gpio motor pins as low
    digitalWrite(lp, LOW);  // left motor
    digitalWrite(rp, LOW);  // right motor

    // initialize pwm properties on motor gpio pins with assigned frequency and resolution  
    const bool lok = ledcAttach(lp, pwmhz, pwmb);   // left motor
    const bool rok = ledcAttach(rp, pwmhz, pwmb);   // right motor

    pwmReady = lok && rok;  // pwm is ready if gpio pin configuration successful

    // initialize startup if pwm setup is successful.
    if (pwmReady) {                 // gpio pins confgured properly

        enterState(State::startup); // initialize startup

    } else {                        // gpio pins weren't configured properly

        enterState(State::fault);   // initialize fault

    }

}

void loop() {

    readTestCommands();

    const State next = nextState(millis());

    if (next != state) {

        enterState(next);

    }

    delay(1);

}

uint32_t dutyFor(int pulseUs) {

    return uint32_t((uint64_t(pulseUs) * pwmhz * (1UL << pwmb) + 500000ULL) / 1000000ULL);

}

int pulseForState(State s) {

    switch(s) {

        case State::forward:
            return fus;
        case State::backward:
            return bus;
        case State::startup: 

        case State::stopped:
            return nus;
        case State::fault:
            return 0;

    }

    return 0;

}

const char* stateName(State s) {

    switch(s) {

        case State::startup:
            return "startup";
        case State::stopped:
            return "stopped";
        case State::forward:
            return "forward";
        case State::backward:
            return "backward";
        case State::fault:
            return "fault";

    }

    return "fault";

}

void disablePWM() {

    ledcDetach(lp);
    ledcDetach(rp);
    pinMode(lp, OUTPUT);
    pinMode(rp, OUTPUT);
    digitalWrite(lp, LOW);
    digitalWrite(rp, LOW);
    pwmReady = false;

}

bool applyOutputs() {

    if (state == State::fault) {

        disablePWM();
        return true;

    }

    if (!pwmReady) {

        return false;

    }

    const uint32_t duty = dutyFor(pulseForState(state));
    const bool lok = ledcWrite(lp, duty);
    const bool rok = ledcWrite(rp, duty);

    return lok && rok;

}

void enterState(State next) {

    state = next;

    if (!applyOutputs()) {

        state = State::fault;
        request = Request::stop;

        applyOutputs();

    }

    enteredAt = millis();
    Serial.println(stateName(state));

}

void readTestCommands() {

    bool stopSeen = false;

    for (unsigned count = 0; count < 64 && Serial.available(); ++count) {

        const char c = char(Serial.read());

        if (state == State::startup || state == State::fault) {

            continue;

        }

        if (c == 's' || c == 'S') {

            request = Request::stop;
            stopSeen = true;

        } else if (!stopSeen && (c == 'f' || c == 'F')) {

            request = Request::forward;

        } else if (!stopSeen && (c == 'b' || c == 'B')) {

            request = Request::backward;

        }

    }

}

State nextState(uint32_t now) {

    const uint32_t elapsed = uint32_t(now - enteredAt);

    switch (state) {

        case State::startup:
            request = Request::stop;
            return elapsed >= startms ? State::stopped : State::startup;

        case State::stopped:
            
            if (elapsed < npausems) {

                return State::stopped;

            }

            if (request == Request::forward) {

                return State::forward;

            }

            if (request == Request::backward) {

                return State::backward;

            }

            return State::stopped;

        case State::forward:
        case State::backward:
            if (elapsed >= runms) {

                request = Request::stop;
                return State::stopped;

            }

            if (request == Request::stop) {

                return State::stopped;

            }

            if (state == State::forward && request == Request::backward) {

                return State::stopped;

            }

            if (request == Request::stop) {

                return State::stopped;

            }

            if (state == State::forward && request == Request::backward) {

                return State::stopped;

            }

            if (state == State::backward && request == Request::forward) {

                return State::stopped;

            }

            return state;

        case State::fault:
        
            request = Request::stop;
            return State::fault;

    }

    return State::fault;

}
