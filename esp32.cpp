// Purpose of program: adapted summer2026 supply drone firmware for 2motor differential steering
    // receive throttle/steering commands over serial and generate left/right esc pwm signals
    // receives numeric serial commands
    // motion uses a test output limit, gradual ramping, a neutral pause before reversal
    // startup, command timeout, stop requests, and centered arming checks gate motion
// Intended SoC: ESP32-WROOM
// Original Dev: Leopold Clark
// Associated Dev(s): Zach Nyairo, Martin Wu
// Date Created: 09/16/2026
// Date Edited: 09/18/2026

// ----dependencies----
#include <Arduino.h> // arduino gpio, serial, timing, and esp32 pwm functions.
#include <esp_arduino_version.h> // esp32 arduino core version definitions - not currently used
#include <stdlib.h> // integer conversion (strtol) and absolute value (abs)
#include <errno.h> // conversion error reporting through errno - for early testing

// ----hardware settings and command state----
// motor commands utilize normalized scale: -1000 reverse, 0 neutral, +1000 forward
// testLimit caps pulse at 25% of full scale ~ 250
constexpr uint8_t leftPin = 25, rightPin = 26, pwmBits = 16; // left/right esc output gpios and 16bit pwm resolution
// 50 Hz esc signal, 400 ms command timeout, and 3s startup
constexpr uint32_t pwmHz = 50, commandTimeoutms = 400, startupms = 3000;
// esc pulse widths in microseconds ~ 250 (.25*1000)
constexpr int reverseUs = 1000, stopUs = 1500, forwardUs = 2000, testLimit = 250;

// keep track of motor commands, timing, and incoming serial messages
bool pwmReady = false, enabled = false, releasedCenter = false, haveCommand = false;
uint32_t started = 0, lastCommand = 0, lastTick = 0, lastReport = 0, commandCount = 0;
int targetLeft = 0, targetRight = 0, leftUs = stopUs, rightUs = stopUs;
char line[64];
size_t lineLength = 0;
bool discardLine = false;

// ----smooth motor changes----
// change each motor command smoothly
// pause at neutral for 300ms before reversing
// stop request goes straight to neutral
struct Motor {

    // each motor remembers current command, last direction, when neutral began
    int value = 0, lastDirection = 0;
    uint32_t neutralSince = 0;

    // repeated stop requests keep original neutral start time
    void stop(uint32_t now) {

        if(value) {

            neutralSince = now;
            value = 0;

        }

    }

    void update(int target, uint32_t now) {

        // temporarily aim for zero when direction change need a pause
        int direction = (target > 0) - (target < 0);
        if(direction && lastDirection && direction != lastDirection && (value || uint32_t(now-neutralSince) < 300)) target = 0;
        int before = value;

        // move toward target in steps of 10 w/o passing
        if(value < target) {

            value += 10;

            if(value > target) {

                value = target;

            }

        }

        if(value > target) {

            value -= 10;
            if(value < target) {

                value = target;

            }

        }

        // recall direction and start pause when command reaches 0
        if(value) {

            lastDirection = (value > 0) - (value < 0);

        } else if (before) {

            neutralSince = now;

        }

    }

};

Motor leftMotor, rightMotor;

// ----stopping and lost commands----
// stop both motors and require enabling
// stop if no valid command arrives > 400 ms
void stopMotors(uint32_t now) {

    enabled = false;
    releasedCenter = false;
    targetLeft = 0;
    targetRight = 0;

    leftMotor.stop(now);
    rightMotor.stop(now);

}

// only valid commands reset timer - bad messages cannot keep motion enabled
void checkTimeout(uint32_t now) {

    if(haveCommand && uint32_t(now - lastCommand) > commandTimeoutms) {

        stopMotors(now);

    }

}

// ----steering----
// combine throttle and steering into left and right motor commands
// keep both commands within test limit
// positive steering raises left command and lowers right command
void mixMotors(int throttle, int steering) {

    int left = throttle + steering, right = throttle - steering;
    int peak = abs(left) > abs(right) ? abs(left) : abs(right);

    // scale both sides together and avoid boosting small stick inputs to limit
    if(peak < 1000) {

        peak = 1000;

    }

    targetLeft = left * testLimit / peak;
    targetRight = right * testLimit / peak;

}

// ----check incoming commands----
// expected format: C, throttle, steering, enable, stop
// throttle and steering: -1000 to 1000
// enable and stop: 0 or 1
// reject messages with bad formatting or values outside these ranges
bool parseCommand(const char* p, int values[4]) {

    if(p[0] != 'C' || p[1] != ',') {

        return false;

    }

    p += 2;

    for(int i = 0; i<4; ++i) {

        if (!(*p == '-' || (*p >= '0' && *p <= '9'))) {

            return false;

        }

        // convert each number and check for errors or values outside its allowed range
        errno = 0;
        char* end = nullptr;
        long n = strtol(p, &end, 10);

        if(errno || end == p || n < (i < 2 ? -1000:0) || n > (i < 2 ? 1000:1)) {

            return false;

        }

        // require commas between values and no extra text after the last value
        if(*end != (i == 3 ? '\0' : ',')) {

            return false;

        }

        values[i] = int(n);
        p = end + (i < 3 ? 1:0);

    }

    return true;

}

// ---allowing motor movement---
// wait 3 seconds after startup
// pwm must be ready and the stop flag must be clear
// enable condition: center both controls, send enable = 0, then enable = 1 while still centered
void acceptCommand(const int v[4], uint32_t now) {

    // check for an expired connection before new message resets the timer
    checkTimeout(now);
    lastCommand = now;
    haveCommand = true;
    ++commandCount;
    bool centered = v[0] == 0 && v[1] == 0;

    if(!pwmReady || v[3] || uint32_t(now - started) < startupms) {

        stopMotors(now);
        return;

    }

    // releasing enable stops both motors and records whether both controls are centered
    if(!v[2]) {

        stopMotors(now);
        releasedCenter = centered;
        return;

    }

    // failed enable attempt requires another centered release before reattempt
    if(!enabled) {

        if(!releasedCenter || !centered) {

            releasedCenter = false;
            return;

        }

        enabled = true;
        releasedCenter = false;

    }

    mixMotors(v[0], v[1]);

}

// ----read serial messages----
// collect one command per line, ending at a newline
// ignore invalid or oversized lines
// read a limited amount each pass so motor updates can keep running
// once a line is invalid or too long discard through next newline
void pollUSB(uint32_t now) {

    for(unsigned budget = 0; budget < 256 && Serial.available(); ++budget) {

        char ch = char(Serial.read());

        if(ch == '\r') {

            continue;

        }

        // process a complete line then clear buffer state for next message
        if(ch == '\n') {

            line[lineLength] = '\0';
            int values[4];

            if(lineLength && !discardLine && parseCommand(line, values)) {

                acceptCommand(values, now);

            }

            lineLength = 0;
            discardLine = false;

        } else if (ch < 32 || ch > 126) {

            discardLine = true;

        } else if (!discardLine) {

            if(lineLength < sizeof(line) - 1) {

                line[lineLength++] = ch;

            } else {

                discardLine = true;

            }

        }

    }

}

// ----convert motor commands to pwm----
// turn motor commands into ESC pulse widths, then into values the PWM hardware uses
// 0 means 1500 microsecond neutral pulse
// current test limit requested pulses between 1375 and 1625 microseconds
int pulseFor(int command) {

    return stopUs + (command >= 0 ? command * (forwardUs - stopUs) : command * (stopUs - reverseUs)) / 1000;

}

// calculate time length of signal high in each pwm cycle rounding to nearest count
uint32_t dutyFor(int us) {

    return uint32_t((uint64_t(us) * pwmHz * (1UL << pwmBits) + 500000ULL) / 1000000ULL);

}

void disablePWM();

// ----send commands to the motors----
// update both esc signals if either pwm write fails stop and disable both outputs
void writeMotors(uint32_t now) {

    if(!pwmReady) {

        return;

    }

    leftUs = pulseFor(leftMotor.value);
    rightUs = pulseFor(rightMotor.value);

    bool leftOk = ledcWrite(leftPin, dutyFor(leftUs));
    bool rightOk = ledcWrite(rightPin, dutyFor(rightUs));

    if(!leftOk || !rightOk) {

        stopMotors(now);
        disablePWM();

    }

}

// turn off pwm and hold both signal pins low until restart
// report zero pulse widths showing no pulses being sent
void disablePWM() {

    ledcDetach(leftPin);
    ledcDetach(rightPin);
    pinMode(leftPin, OUTPUT);
    digitalWrite(leftPin, LOW);
    pinMode(rightPin, OUTPUT);
    digitalWrite(rightPin, LOW);
    pwmReady = false;
    leftUs = 0;
    rightUs = 0;

}

// ----startup----
// start serial communication set up both pwm outputs and send neutral pulses
void setup() {

    Serial.begin(115200);
    pinMode(leftPin, OUTPUT);
    digitalWrite(leftPin, LOW);
    pinMode(rightPin, OUTPUT);
    digitalWrite(rightPin, LOW);
    // both outputs must set up successfully before motor commands can be sent
    bool leftok = ledcAttach(leftPin, pwmHz, pwmBits);
    bool rightok = ledcAttach(rightPin, pwmHz, pwmBits);
    pwmReady = leftok && rightok;

    if(!pwmReady) {

        disablePWM();

    }

    started = lastTick = millis();
    writeMotors(started);

}

// check lost commands, read new ones, and update motor commands every 20 ms
// attempt send status every 100 ms: ack command_count enabled left_us right_us pwm_ready
void loop() {

    uint32_t now = millis();

    checkTimeout(now);
    pollUSB(now);

    if(uint32_t(now - lastTick) >= 20) {

        lastTick = now;
        leftMotor.update(targetLeft, now);
        rightMotor.update(targetRight, now);

    }

    writeMotors(now);

    if(uint32_t(now - lastReport) >= 100){

        lastReport = now;
        char report[80];
        int n = snprintf(report, sizeof(report), "ACK %lu %u %d %d %u\n", (unsigned long)commandCount, unsigned(enabled), leftUs, rightUs, unsigned(pwmReady));

        // skip report if cannot fit instead of halting motor control
        if(n > 0 && n < int(sizeof(report)) && Serial.availableForWrite() >= n){

            Serial.write((const uint8_t*)report, n);

        }

    }

    delay(1);

}
