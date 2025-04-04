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
//     SPEEDS (0–255)
// -----------------------
const int MOTOR_SPEED_DEFAULT = 200;
const int MOTOR_SPEED_SLOW    = 100;
const int MOTOR_SPEED_TURN    = 160;

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
const int SERVO_ANGLE_CENTER = 90;  // zero (180 degree servos)
const int SERVO_ANGLE_CfR = 40;  // for each servo (until we zero them properly)
const int SERVO_ANGLE_CfL = 56;
const int SERVO_ANGLE_CbR = 105;
const int SERVO_ANGLE_CbL = 40;
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

  // show commands
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("steerL[angle] steerR[angle] steerZ | driveF driveB DdriveL DdriveR | S C");
  Serial.println("----------------------------------------------------------------------------");
}

// ---------------------------------------------------------
//                  MAIN PROGRAM LOOP
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
      driveF();
    } else if (command == "driveB") {
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
  servofR.write(SERVO_ANGLE_CfR - angle);
  servobL.write(SERVO_ANGLE_CbL + angle);
  servofL.write(SERVO_ANGLE_CfL - angle);
  servobR.write(SERVO_ANGLE_CbR + angle);
}

void steerR(int angle) {
  servofR.write(SERVO_ANGLE_CfR + angle);
  servobL.write(SERVO_ANGLE_CbL - angle);
  servofL.write(SERVO_ANGLE_CfL + angle);
  servobR.write(SERVO_ANGLE_CbR - angle);
}

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
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, 0);
}

void driveF() {
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, 0);
}

void driveB() {
  SoftPWMSet(MOTOR_LEFT_FORWARD_PIN, MOTOR_SPEED_DEFAULT);
  SoftPWMSet(MOTOR_LEFT_REVERSE_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_FORWARD_PIN, 0);
  SoftPWMSet(MOTOR_RIGHT_REVERSE_PIN, MOTOR_SPEED_DEFAULT);
}

// --------------------------------------------------------------------------
//                      DIFFERENTIAL DRIVE FUNCTIONS
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
