// mapping encoder values to angle range [-90, +90]
#define ABS_ENCODER_TO_DEGREE_VL (map((long)analogRead(encoderVoltagePinVL), 240, 791, -90, 90))  // offset was 514 bits at 0 degree
#define ABS_ENCODER_TO_DEGREE_VR (map((long)analogRead(encoderVoltagePinVR), 237, 779, -90, 90))  // offset was 511 bits at 0 degree
#define ABS_ENCODER_TO_DEGREE_HL (map((long)analogRead(encoderVoltagePinHL), 224, 772, -90, 90))  // offset was 503 bits at 0 degree
#define ABS_ENCODER_TO_DEGREE_HR (map((long)analogRead(encoderVoltagePinHR), 224, 772, -90, 90))  // offset was 500 bits at 0 degree

#include "VescNimbulus.cpp"
#include <TimerOne.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// 9 axis orientation sensor BNO055
// Check I2C device address and correct line below
// (by default address is 0x29 or 0x28)
//                                    id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t BNOevent;
int psi = 0;
int psiOffset;

// End and Safety Stop: Pins & Flags
const int endStopPin = 2;
const int emergencyStopPin = 3;
// flags for ISR must be volatile so they are read from memory
// every time and no deprecated cache value is used
volatile bool endStop = false;
volatile bool emergencyStop = false;
volatile bool sendDataTimeFlag = false;

// digitalized absolute encoder vals converted in angles
int angleVL, angleVR, angleHL, angleHR;
// pins for abs encoders
int encoderVoltagePinVL = A5;
int encoderVoltagePinVR = A13;
int encoderVoltagePinHL = A1;
int encoderVoltagePinHR = A9;

// "turn on the spot" information
int turnOnSpotAngle = 57;
char* turnonSpotString = "<-57, 57, 57, -57>";

// vesc ids
enum vescIDs {
  HL = 0,
  HR = 1,
  VL = 2,
  VR = 3,
};

// debug traffic lights
int debugPinGreen = 48;
int debugPinYellow = 50;
int debugPinRed = 52;

// serial communication input from Matlab to Arduino1
const byte numChars = 25;
char receivedChars[numChars];
bool recvInProgress = false;
byte ndx = 0;
bool newData = false;
const char startMarker = '<';
const char endMarker = '>';
const char* delimiter = ",";
int steeringCommand = 0;
enum steeringCmd {
  NO = 0,   // do not change steering angle
  YES = 1,  // change steering angle
  TURN = 2, // enter "turn on the spot" mode
  POS = 3,  // rotate math postive in "turn on the spot"
  NEG = 4,  // rotate math negative in "turn on the spot"
  END = 5,  // end program (rps = 0, angle = 0, endless loop)
};

// serial communication to matlab output
char outputStringMatlab[100];  // Serial0 to Matlab
char outputStringArdu2[25];    // Serial2 to Arudino2
float rotRateVL;
float rotRateVR;
float rotRateHL;
float rotRateHR;
float rotRate;
char rotRateString[10];
int currentAnglesAveraged;

// variables with received data from matlab
int steeringAngle = 0;
float speedValVL = 0, speedValVR = 0;
float speedValHL = 0, speedValHR = 0;

// vesc controllers
VescNimbulus vescs;

void setup() {
  // Debug Pins
  pinMode(debugPinGreen, OUTPUT);
  pinMode(debugPinYellow, OUTPUT);
  pinMode(debugPinRed, OUTPUT);

  digitalWrite(debugPinGreen, HIGH);
  digitalWrite(debugPinRed, HIGH);

  //////////////// Serial Communication Setup ////////////////
  // Serial0 communication with Matlab
  Serial.begin(115200);
  while (!Serial)
    ;
  // Serial1 communication with VESCs
  vescs.init();  // init vescs i.e. set serial1
  // Serial2 communication with Arduino2
  Serial2.begin(115200);
  while (!Serial2)
    ;

  ////////////////// Initialise the BNO055 sensor ////////////
  if (!bno.begin()) {
    // There was a problem detecting the BNO055
    Serial.println("Ooops, no BNO055 detected ...");
    Serial.println("Check your wiring or I2C ADDR!");
    while (1) // stop in endless loop
      ;
  }
  delay(1000);  // necessary for setup
  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);
  delay(700);  // necessary for setup
  // Get a new sensor event
  bno.getEvent(&BNOevent);
  psiOffset = BNOevent.orientation.x;

  // steering angle
  pinMode(encoderVoltagePinVL, INPUT);
  pinMode(encoderVoltagePinVR, INPUT);
  pinMode(encoderVoltagePinHL, INPUT);
  pinMode(encoderVoltagePinHR, INPUT);

  // steer to 0 everytime code is restarted
  Serial2.print("<0, 0, 0, 0>");
  delay(1000);

  //////////////////////// Attach interrupts /////////////////
  attachInterrupt(digitalPinToInterrupt(endStopPin),
                  handleEndStopISR,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(emergencyStopPin),
                  handleEmergencyStopISR,
                  FALLING);
  Timer1.initialize(500000);  // timer sending data to matlab
  Timer1.attachInterrupt(handleSendingDataISR);

  digitalWrite(debugPinGreen, LOW);
  digitalWrite(debugPinRed, LOW );
}

void loop() {
  digitalWrite(debugPinGreen, HIGH);

  // timerInterrupt controlled:
  // read sensor values and send data from Arduino 1 to MATLAB
  if (sendDataTimeFlag) {
    sendOdometryToMatlab();

    // reset sendDataTimeFlag
    sendDataTimeFlag = false;
  }

  // check end/emergency stops then receive data
  receiveSerialAndCheckSafety();

  if (newData) {  // Check if new data is available
    parseData();  // store new data
    handleTurningCmds();
    newData = false;  // Reset newData flag after processing
  }

  // send rps to vescs every loop
  // because if no data is sent, vescs will stop
  vescs.setRPS(speedValHL, speedValHR,
               speedValVL, speedValVR);
}

void sendOdometryToMatlab() {
  // Get a new BNO055 sensor event
  bno.getEvent(&BNOevent);
  psi = -(((int)BNOevent.orientation.x) - psiOffset);

  // read abs encoder values (current steering angles)
  currentAnglesAveraged =
    (ABS_ENCODER_TO_DEGREE_VL + ABS_ENCODER_TO_DEGREE_VR +
     ABS_ENCODER_TO_DEGREE_HL + ABS_ENCODER_TO_DEGREE_HR) / 4;

  // read VESC values
  rotRateVL = abs(vescs.getRPS(VL));
  rotRateVR = vescs.getRPS(VR);
  rotRateHL = abs(vescs.getRPS(HL));
  rotRateHR = abs(vescs.getRPS(HR));
  // average values and get direction from front right wheel
  // because in "turn on the spot" mode simple averaging
  // should result in approx 0 all the time
  rotRate = (rotRateVR >= 0 ? 1 : -1) *
    (rotRateVL + abs(rotRateVR) + rotRateHL + rotRateHR) / 4;

  // send data to matlab
  dtostrf(rotRate, 6, 3, rotRateString);
  sprintf(outputStringMatlab, "<%lu, %d, %s, %d>",
            millis(), currentAnglesAveraged,
            rotRateString, psi);
  Serial.println(outputStringMatlab);
}

void receiveSerialAndCheckSafety() {
  if (!endStop && !emergencyStop && steeringCommand != END) {
    receiveSerialData();
  } else if (endStop) {
    // at least one wheel steered out of the possible range
    newData = false;
    speedValVL = speedValVR = speedValHL = speedValHR = 0;
    vescs.setRPS(speedValHL, speedValHR,
                 speedValVL, speedValVR);
    Serial2.print("<0, 0, 0, 0>");
    delay(2500); // wait so user can take action
    endStop = false; // reset flag
  } else if (emergencyStop) {
    // emergency remote or button was pressed
    newData = false;
    vescs.setRPS(0, 0, 0, 0);
    while (true) { // stay idle in endless loop
      digitalWrite(debugPinRed, HIGH);
      digitalWrite(debugPinYellow, HIGH);
      digitalWrite(debugPinGreen, HIGH);
      delay(100);
      digitalWrite(debugPinRed, LOW);
      digitalWrite(debugPinYellow, LOW);
      digitalWrite(debugPinGreen, LOW);
      delay(100);
    }
  }
}

// non blocking read of serial input string delimited
// by '<' and '>'. receivedChars does not contain
// start and end markers after complete receive
void receiveSerialData() {
  while (Serial.available() > 0 && newData == false) {
    char rc = Serial.read();

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

  // start string tokenizing
  strtokIndx = strtok(receivedChars, delimiter);
  // read steering command as int
  steeringCommand = atoi(strtokIndx);

  // continue tokenizing
  strtokIndx = strtok(NULL, delimiter);
  // read steering angle as int
  steeringAngle = atoi(strtokIndx);

  // continue tokenizing
  strtokIndx = strtok(NULL, delimiter);
  // read speedVal as float
  speedValVL = speedValVR = speedValHL = speedValHR
    = atof(strtokIndx);
}

void handleTurningCmds() {
  switch (steeringCommand) {
    case NO:  // no additional action required
      return;
    case YES: // trigger steering and set command to NO
      sprintf(outputStringArdu2, "<%d, %d, %d, %d>",
                steeringAngle, steeringAngle,
                steeringAngle, steeringAngle);
      Serial2.print(outputStringArdu2);
      steeringCommand = NO;
      return;
    case TURN: // steer to turnOnSpotAngle and stop wheels
      Serial2.print(turnonSpotString);
      speedValVL = speedValVR = speedValHL = speedValHR = 0;
      return;
    case POS: // rotate math positive (counter clockwise)
      speedValHL = speedValVL = -1;
      speedValHR = speedValVR = 1;
      return;
    case NEG: // rotate math negative (clockwise)
      speedValHR = speedValVR = -1;
      speedValHL = speedValVL = 1;
      return;
    case END:
      vescs.setRPS(0, 0, 0, 0); // stop now
      Serial2.print("<0, 0, 0, 0>"); // steer to zero
      while (true) { // end idle in endless loop
        digitalWrite(debugPinRed, HIGH);
        digitalWrite(debugPinYellow, LOW);
        digitalWrite(debugPinGreen, HIGH);
        delay(100);
        digitalWrite(debugPinRed, LOW);
        digitalWrite(debugPinYellow, HIGH);
        digitalWrite(debugPinGreen, LOW);
        delay(100);
      }
    default: // should never be reached
      return;
  }
}

// ISR for end stop
void handleEndStopISR() {
  endStop = true;
}

// ISR for emergeny stop
void handleEmergencyStopISR() {
  emergencyStop = true;
}

// ISR sending data to matlab
void handleSendingDataISR() {
  sendDataTimeFlag = true;
}
