// mapping encoder values to angle range [-90, +90]
#define ABS_ENCODER_TO_DEGREE_VL (map((long)analogRead(encoderVoltagePinVL), 252, 847, -90, 90))  // offset was 552 bits at 0 degree
#define ABS_ENCODER_TO_DEGREE_VR (map((long)analogRead(encoderVoltagePinVR), 247, 836, -90, 90))  // offset was 550 bits at 0 degree
#define ABS_ENCODER_TO_DEGREE_HL (map((long)analogRead(encoderVoltagePinHL), 233, 829, -90, 90))  // offset was 540 bits at 0 degree
#define ABS_ENCODER_TO_DEGREE_HR (map((long)analogRead(encoderVoltagePinHR), 233, 828, -90, 90))  // offset was 537 bits at 0 degree


// digitalized absolute encoder values converted in angles
int angleVL, angleVR, angleHL, angleHR;
// pins for abs encoders
int encoderVoltagePinVL = A5;
int encoderVoltagePinVR = A13;
int encoderVoltagePinHL = A1;
int encoderVoltagePinHR = A9;

// upper bound for absolute angle
int maxAngle = 95;
int tolerance = 5;  // angle in +- tolerance

//steering
int steeringDirectionPinVL = 40;
int pulPinVL = 46;
int steeringDirectionPinVR = 34;
int pulPinVR = 48;
int steeringDirectionPinHL = 36;
int pulPinHL = 44;
int steeringDirectionPinHR = 38;
int pulPinHR = 42;

int firstStepsVL, firstStepsVR, firstStepsHL, firstStepsHR;
int stepsThreshold = 10;
int minDelay = 250;
int maxDelay = 1500;
int delayValue;

// serial communication input from ardu 1
const byte numChars = 25;
char receivedChars[numChars];
bool recvInProgress = false;
byte ndx = 0;
bool newData = false;
const char startMarker = '<';
const char endMarker = '>';
const char* delimiter = ",";

// variables with received data
int steeringAngleVL = 0;
int steeringAngleVR = 0;
int steeringAngleHL = 0;
int steeringAngleHR = 0;

void setup() {
  // serial0 to computer (for debugging if desired)
  Serial.begin(115200);
  while(!Serial);
  Serial2.begin(115200); // Serial2 to ardu 1
  while (!Serial2)
    ;

  // steering
  pinMode(steeringDirectionPinVL, OUTPUT);
  pinMode(pulPinVL, OUTPUT);
  pinMode(steeringDirectionPinVR, OUTPUT);
  pinMode(pulPinVR, OUTPUT);
  pinMode(steeringDirectionPinHL, OUTPUT);
  pinMode(pulPinHL, OUTPUT);
  pinMode(steeringDirectionPinHR, OUTPUT);
  pinMode(pulPinHR, OUTPUT);

  // steering angle
  pinMode(encoderVoltagePinVL, INPUT);
  pinMode(encoderVoltagePinVR, INPUT);
  pinMode(encoderVoltagePinHL, INPUT);
  pinMode(encoderVoltagePinHR, INPUT);

  // steer to 0
  steerFourWheels();
}

void loop() {
  receiveSerialData();  // Receive serial data

  if (newData) {  // Check if new data is available
    parseData();
    newData = false;  // Reset newData flag after processing
    limitSteeringAngle();
    steerFourWheels();
  }
}

// non blocking read of serial input string delimited
// by '<' and '>'. receivedChars does not contain
// start and end markers after complete receive
void receiveSerialData() {
  while (Serial2.available() > 0 && newData == false) {
    char rc = Serial2.read();

    if (recvInProgress) { // '<' already received before
      if (rc != endMarker) { // append read char
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          // input too long, write in last available char
          ndx = numChars - 1;
        }
      } else { // discard '>' char and terminate string
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false; // reset flag
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      // discard '<' char
      recvInProgress = true;
    } // ignore data not starting with '<'
  }
}

// store received data in variables
void parseData() {
  char* strtokIndx;

  // parse all 4 steering angles
  strtokIndx = strtok(receivedChars, delimiter);
  steeringAngleVL = atoi(strtokIndx);
  strtokIndx = strtok(NULL, delimiter);
  steeringAngleVR = atoi(strtokIndx);
  strtokIndx = strtok(NULL, delimiter);
  steeringAngleHL = atoi(strtokIndx);
  strtokIndx = strtok(NULL, delimiter);
  steeringAngleHR = atoi(strtokIndx);
}

void limitSteeringAngle() {
  // limit steering angles to maxAngle while keeping direction
  if (abs(steeringAngleVL) > maxAngle) {
    steeringAngleVL =
      abs(steeringAngleVL) * maxAngle / steeringAngleVL;
  }
  if (abs(steeringAngleVR) > maxAngle) {
    steeringAngleVR =
      abs(steeringAngleVR) * maxAngle / steeringAngleVR;
  }
  if (abs(steeringAngleHL) > maxAngle) {
    steeringAngleHL =
      abs(steeringAngleHL) * maxAngle / steeringAngleHL;
  }
  if (abs(steeringAngleHR) > maxAngle) {
    steeringAngleHR =
      abs(steeringAngleHR) * maxAngle / steeringAngleHR;
  }
}

void steerFourWheels() {
  // read encoder voltage and translate to degree
  angleVL = ABS_ENCODER_TO_DEGREE_VL;
  angleVR = ABS_ENCODER_TO_DEGREE_VR;
  angleHL = ABS_ENCODER_TO_DEGREE_HL;
  angleHR = ABS_ENCODER_TO_DEGREE_HR;

  // initialize firstSteps to zero
  firstStepsVL = firstStepsVR = 0;
  firstStepsHL = firstStepsHR = 0;

  // calc the amount of steps per each wheel (angle * 10)
  int stepsVL = abs(angleVL - steeringAngleVL) * 10.0;
  int stepsVR = abs(angleVR - steeringAngleVR) * 10.0;
  int stepsHL = abs(angleHL - steeringAngleHL) * 10.0;
  int stepsHR = abs(angleHR - steeringAngleHR) * 10.0;

  while (stepsVL + stepsVR + stepsHL + stepsHR > 0) {
    // check for Serial2 data
    if (Serial2.available()) {
      // stop on new incoming angles by reducing number of
      // steps to stepsThreshold
      stepsVL = min(stepsVL, stepsThreshold);
      stepsVR = min(stepsVR, stepsThreshold);
      stepsHL = min(stepsHL, stepsThreshold);
      stepsHR = min(stepsHR, stepsThreshold);
    }

    // make steps for each wheel in turn so that seemingly
    // parallel steering is achieved
    if (stepsVL != 0) {
      wheelPositionStep(steeringDirectionPinVL, pulPinVL,
                        firstStepsVL, stepsVL,
                        steeringAngleVL, angleVL);
      firstStepsVL++;
      stepsVL--;
    } else {
      delayMicroseconds(minDelay);
    }

    if (stepsVR != 0) {
      wheelPositionStep(steeringDirectionPinVR, pulPinVR,
                        firstStepsVR, stepsVR,
                        steeringAngleVR, angleVR);
      firstStepsVR++;
      stepsVR--;
    } else {
      delayMicroseconds(minDelay);
    }

    if (stepsHL != 0) {
      wheelPositionStep(steeringDirectionPinHL, pulPinHL,
                        firstStepsHL, stepsHL,
                        steeringAngleHL, angleHL);
      firstStepsHL++;
      stepsHL--;
    } else {
      delayMicroseconds(minDelay);
    }

    if (stepsHR != 0) {
      wheelPositionStep(steeringDirectionPinHR, pulPinHR,
                        firstStepsHR, stepsHR,
                        steeringAngleHR, angleHR);
      firstStepsHR++;
      stepsHR--;
    } else {
      delayMicroseconds(minDelay);
    }
  }
}

void wheelPositionStep(int steeringDirPin, int pulPin,
                       int firstSteps, int remainingSteps,
                       int steeringAngle, int angle) {
  delayValue = minDelay;
  // map delayValue at the borders of the interval so that
  // beginning and end of steering is not too abrupt
  if (firstSteps < stepsThreshold) {
    delayValue = map(firstSteps,
                     0, stepsThreshold,
                     maxDelay, minDelay);
  } else if (remainingSteps < stepsThreshold) {
    delayValue = map(remainingSteps,
                     stepsThreshold, 0,
                     minDelay, maxDelay);
  }

  // set direction pin correctly
  if (angle > steeringAngle) {
    digitalWrite(steeringDirPin, HIGH);
  } else if (angle < steeringAngle) {
    digitalWrite(steeringDirPin, LOW);
  }

  digitalWrite(pulPin, HIGH);
  delayMicroseconds(25);
  digitalWrite(pulPin, LOW);
  delayMicroseconds(delayValue);
}
