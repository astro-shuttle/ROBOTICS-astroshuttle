#include <Servo.h>
#include <SoftPWM.h>

// -----------------------
//     DC MOTOR PINS
// -----------------------
const int MOTOR_LEFT_FORWARD_PIN  = 2;  // left motor "forward" channel
const int MOTOR_LEFT_REVERSE_PIN  = 3;  // left motor "reverse" channel
const int MOTOR_RIGHT_FORWARD_PIN = 4;  // right motor "forward" channel
const int MOTOR_RIGHT_REVERSE_PIN = 5;  // right motor "reverse" channel

// -----------------------
//     SPEEDS (0?255)
// -----------------------
const int MOTOR_SPEED_DEFAULT = 50;
const int MOTOR_SPEED_SLOW    = MOTOR_SPEED_DEFAULT * 0.5;
const int MOTOR_SPEED_TURN    = MOTOR_SPEED_DEFAULT * 0.8;

// -----------------------
//       SERVO PINS
// -----------------------
const int SERVO_FR_PIN = 8;   // Front Right servo
const int SERVO_FL_PIN = 11;  // Front Left servo
const int SERVO_BL_PIN = 10;  // Back Left servo
const int SERVO_BR_PIN = 9;   // Back Right servo

// -----------------------
//     SERVO ANGLES
// -----------------------
const int SERVO_ANGLE_CENTER = 90;  // Zero (180 degree servos)
const int SERVO_ANGLE_FB_BEARING = 15; // For driveF and driveB
int SERVO_ANGLE_CfR = SERVO_ANGLE_CENTER;
int SERVO_ANGLE_CfL = SERVO_ANGLE_CENTER;
int SERVO_ANGLE_CbR = SERVO_ANGLE_CENTER;
int SERVO_ANGLE_CbL = SERVO_ANGLE_CENTER;
const int SERVO_ANGLE_TURN   = 30;  // Offset for servo-assisted turns
const int SERVO_ANGLE_PIVOT  = 40;  // Additional offset for pivot turns

// -----------------------
//     SERVO OBJECTS
// -----------------------
Servo servofR;
Servo servofL;
Servo servobL;
Servo servobR;

// ---------------------------------------------------------
//                          SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(9600);
  
  // set motor driver pins as outputs
  pinMode(MOTOR_LEFT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_REVERSE_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_REVERSE_PIN, OUTPUT);
  
  // initialize SoftPWM
  SoftPWMBegin();
  
  // attach servos to pins
  servofR.attach(SERVO_FR_PIN);
  servofL.attach(SERVO_FL_PIN);
  servobL.attach(SERVO_BL_PIN);
  servobR.attach(SERVO_BR_PIN);

  // RESET OBERON
  steerZ();
  stopMotors();

  // ENABLE PINS 
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // Display available commands on Serial Monitor
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("                             Oberon INITIALIZED");
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("steerL[angle] steerR[angle] steerZ | driveF driveB DdriveL DdriveR | S C");
  Serial.println("----------------------------------------------------------------------------");
}


// ---------------------------------------------------------
//                MAIN PROGRAM LOOP
// ---------------------------------------------------------
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // SERVO COMMANDS WITH ATTACHED ANGLES
    if (command.startsWith("steerL ")) {
      String angleStr = command.substring(7); // extract angle after "steerL "
      int angle = angleStr.toInt();
      steerL(angle);
    }
    else if (command.startsWith("steerR ")) {
      String angleStr = command.substring(7);
      int angle = angleStr.toInt();
      steerR(angle);
    }
    // DEFAULT SERVO COMMANDS
    else if (command == "steerL") {
      steerL();  // default steer left
    } else if (command == "steerR") {
      steerR();  // default steer right
    } else if (command == "steerZ") {
      steerZ();
    }
    // DRIVE COMMANDS
    else if (command == "driveF") {

      SERVO_ANGLE_CfR = SERVO_ANGLE_CENTER + SERVO_ANGLE_FB_BEARING;
      SERVO_ANGLE_CfL = SERVO_ANGLE_CENTER - SERVO_ANGLE_FB_BEARING;
      SERVO_ANGLE_CbR = SERVO_ANGLE_CENTER + SERVO_ANGLE_FB_BEARING;
      SERVO_ANGLE_CbL = SERVO_ANGLE_CENTER - SERVO_ANGLE_FB_BEARING;
      
      driveF();
    } else if (command == "driveB") {

      SERVO_ANGLE_CfR = SERVO_ANGLE_CENTER - SERVO_ANGLE_FB_BEARING;
      SERVO_ANGLE_CfL = SERVO_ANGLE_CENTER + SERVO_ANGLE_FB_BEARING;
      SERVO_ANGLE_CbR = SERVO_ANGLE_CENTER - SERVO_ANGLE_FB_BEARING;
      SERVO_ANGLE_CbL = SERVO_ANGLE_CENTER + SERVO_ANGLE_FB_BEARING;
      
      driveB();
    } else if (command == "DdriveL") {
      DdriveL();
    } else if (command == "DdriveR") {
      DdriveR();
    }
    // Stop motors
    else if (command == "S") {
      stopMotors();
    }
    // Show commands
    else if (command == "C") {
      showCommands();
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
    
    Serial.print("Executed command: ");
    Serial.println(command);
  }
}

// ---------------------------------------------------------
//                  COMMAND FUNCTIONS
// ---------------------------------------------------------

// Display the list of commands
void showCommands() {
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("steerL[angle] steerR[angle] steerZ | driveF driveB DdriveL DdriveR | S C");
  Serial.println("----------------------------------------------------------------------------");
}
// ---------------------------------------------------------
//                    SERVO FUNCTIONS
// ---------------------------------------------------------
void steerL(int angle) {
  // Adjust servos to steer left using specified angle offset.
  servofR.write(SERVO_ANGLE_CfR - angle);
  servobL.write(SERVO_ANGLE_CbL + angle);
  servofL.write(SERVO_ANGLE_CfL - angle);
  servobR.write(SERVO_ANGLE_CbR + angle);
}

void steerR(int angle) {
  // Adjust servos to steer right using specified angle offset.
  servofR.write(SERVO_ANGLE_CfR + angle);
  servobL.write(SERVO_ANGLE_CbL - angle);
  servofL.write(SERVO_ANGLE_CfL + angle);
  servobR.write(SERVO_ANGLE_CbR - angle);
}

// Default versions using SERVO_ANGLE_TURN:
void steerL() {
  steerL(SERVO_ANGLE_TURN);
}

void steerR() {
  steerR(SERVO_ANGLE_TURN);
}
void steerZ() {
  servofR.write(SERVO_ANGLE_CfR);
  servobL.write(SERVO_ANGLE_CbL);
  servofL.write(SERVO_ANGLE_CfL);
  servobR.write(SERVO_ANGLE_CbR);
}
// ---------------------------------------------------------
//                   DC MOTOR FUNCTIONS
// ---------------------------------------------------------
void stopMotors() {
  // Stop all motors by setting both channels to zero speed
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, 0);
}

void driveF() {
  // Drive forward: activate forward channels
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, 0);
}

void driveB() {
  // Drive backward: activate reverse channels
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, MOTOR_SPEED_DEFAULT);
}

void skidL() {
  // Skid steer left (in-place turn): left motor forward, right motor reverse
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, MOTOR_SPEED_TURN);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, MOTOR_SPEED_TURN);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, 0);
}

void skidR() {
  // Skid steer right (in-place turn): left motor reverse, right motor forward
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, MOTOR_SPEED_TURN);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, MOTOR_SPEED_TURN);
}

// --------------------------------------------------------------------------
//                           COMBINED FUNCTIONS
// --------------------------------------------------------------------------
void DdriveL() {
  steerL();
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, MOTOR_SPEED_SLOW);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, 0);
}

void DdriveR() {
  steerR();
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, MOTOR_SPEED_SLOW);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, 0);
}
