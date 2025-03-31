#include <Servo.h>

// -----------------------
//  MOTOR CONTROLLER PINS (goBILDA ESC-style)
// -----------------------
const int MOTOR_LEFT_CTRL_PIN  = 2;  // Left motor controller signal
const int MOTOR_RIGHT_CTRL_PIN = 3;  // Right motor controller signal

// -----------------------
//  MOTOR SPEED CONSTANTS (servo values: 0-180, with 90 = stop)
// -----------------------
const int MOTOR_STOP            = 90;  // Neutral (motor off)
const int MOTOR_FORWARD_DEFAULT = 180; // Full forward
const int MOTOR_FORWARD_SLOW    = 140; // Slow forward
const int MOTOR_FORWARD_TURN    = 160; // For skid/tight turns
const int MOTOR_REVERSE_DEFAULT = 0;   // Full reverse

// -----------------------
//  STEERING SERVO PINS
// -----------------------
const int SERVO_FR_PIN = 8;   // Front Right servo
const int SERVO_FL_PIN = 11;  // Front Left servo
const int SERVO_BL_PIN = 10;  // Back Left servo
const int SERVO_BR_PIN = 9;   // Back Right servo

// -----------------------
//  STEERING SERVO ANGLES (custom centers and offsets)
// -----------------------
const int SERVO_ANGLE_CENTER = 90;  // Center position for steering servos
const int SERVO_ANGLE_CfR = 90;       // Custom center for front right servo
const int SERVO_ANGLE_CfL = 90;       // Custom center for front left servo
const int SERVO_ANGLE_CbR = 90;       // Custom center for back right servo
const int SERVO_ANGLE_CbL = 90;       // Custom center for back left servo
const int SERVO_ANGLE_TURN   = 30;    // Default offset for servo-assisted turns
const int SERVO_ANGLE_PIVOT  = 40;    // Additional offset for pivot turns

// -----------------------
//  SERVO OBJECTS
// -----------------------
Servo leftMotor;   // Motor controller for left drive motor
Servo rightMotor;  // Motor controller for right drive motor
Servo servofR;     // Steering: front right wheel
Servo servofL;     // Steering: front left wheel
Servo servobL;     // Steering: back left wheel
Servo servobR;     // Steering: back right wheel

// ---------------------------------------------------------
//                          SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(9600);
  
  // Attach motor controller signals
  leftMotor.attach(MOTOR_LEFT_CTRL_PIN);
  rightMotor.attach(MOTOR_RIGHT_CTRL_PIN);
  
  // Attach steering servos
  servofR.attach(SERVO_FR_PIN);
  servofL.attach(SERVO_FL_PIN);
  servobL.attach(SERVO_BL_PIN);
  servobR.attach(SERVO_BR_PIN);

  // Reset: center steering and stop motors
  steerZ();
  stopMotors();
  
  // Display available commands
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("                             Oberon INITIALIZED");
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("steerL[angle] steerR[angle] steerZ | driveF driveB skidL skidR DdriveL DdriveR | S C");
  Serial.println("----------------------------------------------------------------------------");
}

// ---------------------------------------------------------
//                MAIN PROGRAM LOOP
// ---------------------------------------------------------
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // --- Steering commands with attached angle parameter ---
    if (command.startsWith("steerL ")) {
      String angleStr = command.substring(7); // Get the angle (after "steerL ")
      int angle = angleStr.toInt();
      steerL(angle);
    }
    else if (command.startsWith("steerR ")) {
      String angleStr = command.substring(7);
      int angle = angleStr.toInt();
      steerR(angle);
    }
    // --- Default steering commands ---
    else if (command == "steerL") {
      steerL();  // Use default turn offset
    } else if (command == "steerR") {
      steerR();
    } else if (command == "steerZ") {
      steerZ();
    }
    // --- Drive commands ---
    else if (command == "driveF") {
      driveF();
    } else if (command == "driveB") {
      driveB();
    } else if (command == "skidL") {
      skidL();
    } else if (command == "skidR") {
      skidR();
    } else if (command == "DdriveL") {
      DdriveL();
    } else if (command == "DdriveR") {
      DdriveR();
    }
    // --- Stop motors ---
    else if (command == "S") {
      stopMotors();
    }
    // --- Show command list ---
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
void showCommands() {
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("steerL[angle] steerR[angle] steerZ | driveF driveB skidL skidR DdriveL DdriveR | S C");
  Serial.println("----------------------------------------------------------------------------");
}

// ---------------------------------------------------------
//                    STEERING SERVO FUNCTIONS
// ---------------------------------------------------------
void steerL(int angle) {
  // Adjust steering servos to turn left by the specified offset.
  servofR.write(SERVO_ANGLE_CfR + angle);
  servobL.write(SERVO_ANGLE_CbL + angle);
  servofL.write(SERVO_ANGLE_CfL - angle);
  servobR.write(SERVO_ANGLE_CbR - angle);
}

void steerR(int angle) {
  // Adjust steering servos to turn right by the specified offset.
  servofR.write(SERVO_ANGLE_CfR - angle);
  servobL.write(SERVO_ANGLE_CbL - angle);
  servofL.write(SERVO_ANGLE_CfL + angle);
  servobR.write(SERVO_ANGLE_CbR + angle);
}

// Default versions using SERVO_ANGLE_TURN:
void steerL() {
  steerL(SERVO_ANGLE_TURN);
}

void steerR() {
  steerR(SERVO_ANGLE_TURN);
}

void steerZ() {
  // Center all steering servos.
  servofR.write(SERVO_ANGLE_CfR);
  servofL.write(SERVO_ANGLE_CfL);
  servobL.write(SERVO_ANGLE_CbL);
  servobR.write(SERVO_ANGLE_CbR);
}

// ---------------------------------------------------------
//                   MOTOR CONTROL FUNCTIONS
//    (Using goBILDA 1x15A as RC ESCs via the Servo library)
// ---------------------------------------------------------
void stopMotors() {
  // Set both motor controllers to neutral (stop)
  leftMotor.write(MOTOR_STOP);
  rightMotor.write(MOTOR_STOP);
}

void driveF() {
  // Drive forward: both motors at full forward speed.
  leftMotor.write(MOTOR_FORWARD_DEFAULT);
  rightMotor.write(MOTOR_FORWARD_DEFAULT);
}

void driveB() {
  // Drive backward: both motors at full reverse.
  leftMotor.write(MOTOR_REVERSE_DEFAULT);
  rightMotor.write(MOTOR_REVERSE_DEFAULT);
}

void skidL() {
  // Skid steer left (in-place turn): left motor reverse, right motor forward.
  leftMotor.write(MOTOR_REVERSE_DEFAULT);
  rightMotor.write(MOTOR_FORWARD_DEFAULT);
}

void skidR() {
  // Skid steer right (in-place turn): left motor forward, right motor reverse.
  leftMotor.write(MOTOR_FORWARD_DEFAULT);
  rightMotor.write(MOTOR_REVERSE_DEFAULT);
}

void DdriveL() {
  // Differential drive left: adjust steering then set left motor slower.
  steerL();
  leftMotor.write(MOTOR_FORWARD_SLOW);
  rightMotor.write(MOTOR_FORWARD_DEFAULT);
}

void DdriveR() {
  // Differential drive right: adjust steering then set right motor slower.
  steerR();
  leftMotor.write(MOTOR_FORWARD_DEFAULT);
  rightMotor.write(MOTOR_FORWARD_SLOW);
}
