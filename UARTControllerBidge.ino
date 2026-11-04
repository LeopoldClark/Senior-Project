// motor and servo control using serial commands
// format: CLLRR or OLLRR followed by newline
// C = closed, O = open
// LL = left motor command, RR = right motor command
// motor commands range from 00 to 99

// motor pulse values in microseconds
const int stopPWM = 1000;
const int forwardPWM = 1125;

// motor trims
const float leftTrim = 1.05;
const float rightTrim = 3.75;

// signal pins
const int leftMotorPin = 25; // gpio values for each mechanical component
const int rightMotorPin = 26;
const int servo1 = 18;
const int servo2 = 19;

// servo targets in microseconds
// small test movements; replace with verified arm positions
const int servo1Closed = 500;
const int servo1Open = 2000;
const int servo2Closed = 2500;
const int servo2Open = 1000;

// PWM configuration
const int pwmF = 50;
const int pwmRes = 16;

// received commands
char armCmd = 'C';
int lCmd = 0;
int rCmd = 0;

// function declarations
void readCmds();
void motorPulse();
void servoPulse();
int pulseToDuty(int pulseWidth);

int pulseToDuty(int pulseWidth){

  // conversion for 50 Hz and 16-bit resolution
  return int((pulseWidth/20000.0)*65536.0+0.5);

}

void readCmds(){

  if(Serial.available() > 0){

    String message = Serial.readStringUntil('\n');
    message.trim();

    // check message length and arm command
    if(message.length() != 5){return;}

    if(message.charAt(0) != 'C' && message.charAt(0) != 'O'){
      return;
    }

    // require four decimal digits
    for(int i = 1; i < 5; i++){

      char digit = message.charAt(i);

      if(digit < '0' || digit > '9'){return;}

    }

    // update commands only after validation
    armCmd = message.charAt(0);
    lCmd = message.substring(1, 3).toInt();
    rCmd = message.substring(3, 5).toInt();

    // echo accepted command
    Serial.printf("%c%02d%02d\n", armCmd, lCmd, rCmd);

  }

}

void motorPulse(){

  // convert 00-99 into fractions of the command range
  float left = lCmd/99.0;
  float right = rCmd/99.0;

  // retain the existing motor trims
  left = left*leftTrim;
  right = right*rightTrim;

  // convert trimmed commands to pulse widths
  int lPulse = stopPWM + int(left*(forwardPWM-stopPWM)+0.5);
  int rPulse = stopPWM + int(right*(forwardPWM-stopPWM)+0.5);

  // send commands to the ESCs
  ledcWrite(leftMotorPin, pulseToDuty(lPulse));
  ledcWrite(rightMotorPin, pulseToDuty(rPulse));

}

void servoPulse(){

  int servo1Pulse = servo1Closed;
  int servo2Pulse = servo2Closed;

  // select open positions when requested
  if(armCmd == 'O'){

    servo1Pulse = servo1Open;
    servo2Pulse = servo2Open;

  }

  // send commands to the servos
  ledcWrite(servo1, pulseToDuty(servo1Pulse));
  ledcWrite(servo2, pulseToDuty(servo2Pulse));

}

void setup(){

  Serial.begin(115200);
  Serial.setTimeout(50);

  // configure motor signal outputs
  ledcAttach(leftMotorPin, pwmF, pwmRes);
  ledcAttach(rightMotorPin, pwmF, pwmRes);

  // begin with stopped motors
  motorPulse();

  // configure servo signal outputs
  ledcAttach(servo1, pwmF, pwmRes);
  ledcAttach(servo2, pwmF, pwmRes);

  // begin at closed target positions
  servoPulse();

}

void loop(){

  readCmds();
  motorPulse();
  servoPulse();

  delay(50);

}