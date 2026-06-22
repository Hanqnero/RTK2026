/**
 * MeArm Servo Controller Arduino Sketch
 * 
 * Simple serial communication for servo control
 * Compatible with ROS2 mearm_driver
 * 
 * Protocol: 
 * - Receives commands: [0xFF, servo1, servo2, servo3, servo4]
 * - servo values: 0-180 degrees (PWM width mapped to servo angle)
 * 
 * Pins:
 * - Servo 1 (base rotation): Pin 9
 * - Servo 2 (right arm): Pin 10
 * - Servo 3 (left arm): Pin 11
 * - Servo 4 (gripper): Pin 12
 */

#include <Servo.h>

// Servo objects for 4 servos
Servo servo1;  // Base rotation
Servo servo2;  // Right arm
Servo servo3;  // Left arm
Servo servo4;  // Gripper

// Servo pins
const int SERVO_PINS[4] = {9, 10, 11, 12};
Servo servos[4];

// Current servo angles (0-180 degrees)
int servoAngles[4] = {90, 90, 90, 90};

// Serial communication parameters
const long BAUD_RATE = 115200;
const byte START_MARKER = 0xFF;
const byte COMMAND_LENGTH = 5;  // 1 byte marker + 4 servo values

// Timing
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 50;  // 20 Hz update rate

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  delay(1000);  // Wait for serial to be ready
  
  // Attach servos to pins
  for (int i = 0; i < 4; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(servoAngles[i]);  // Set to neutral position
  }
  
  // Send startup message
  Serial.println("MeArm Controller Ready");
  Serial.print("Baud: ");
  Serial.println(BAUD_RATE);
  
  lastUpdateTime = millis();
}

void loop() {
  // Process incoming serial commands
  handleSerialCommands();
  
  // Periodic status update
  if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = millis();
    // Can add status reporting here if needed
  }
}

/**
 * Handle incoming serial commands
 * Expected format: 0xFF + 4 servo angles
 */
void handleSerialCommands() {
  static byte buffer[COMMAND_LENGTH];
  static int bufferIndex = 0;
  
  while (Serial.available() > 0) {
    byte incoming = Serial.read();
    
    // Check for start marker
    if (incoming == START_MARKER) {
      bufferIndex = 0;
      buffer[bufferIndex] = incoming;
      bufferIndex++;
    } else if (bufferIndex > 0) {
      // Fill command buffer
      buffer[bufferIndex] = incoming;
      bufferIndex++;
      
      // Check if we have complete command
      if (bufferIndex >= COMMAND_LENGTH) {
        processServoCommand(buffer);
        bufferIndex = 0;  // Reset for next command
      }
    }
  }
}

/**
 * Process servo command
 * Command format: [0xFF, servo1_angle, servo2_angle, servo3_angle, servo4_angle]
 * servo*_angle: 0-180 degrees
 */
void processServoCommand(const byte* command) {
  // Validate start marker
  if (command[0] != START_MARKER) {
    return;
  }
  
  // Extract and validate servo angles
  for (int i = 0; i < 4; i++) {
    byte angle = command[i + 1];
    
    // Clamp angle to 0-180 range
    if (angle > 180) {
      angle = 180;
    }
    
    servoAngles[i] = angle;
  }
  
  // Write to servos
  updateServos();
  
  // Optional: Send confirmation (can be disabled to reduce serial traffic)
  // sendFeedback();
}

/**
 * Update servo positions
 */
void updateServos() {
  for (int i = 0; i < 4; i++) {
    servos[i].write(servoAngles[i]);
  }
}

/**
 * Send feedback to host (optional)
 * Can be used to confirm command reception
 */
void sendFeedback() {
  Serial.write(START_MARKER);
  for (int i = 0; i < 4; i++) {
    Serial.write(servoAngles[i]);
  }
}

/**
 * Convenience function: Set all servos to neutral position
 */
void setNeutral() {
  for (int i = 0; i < 4; i++) {
    servoAngles[i] = 90;
  }
  updateServos();
}

/**
 * Convenience function: Stow arm (retract)
 */
void stowArm() {
  servoAngles[0] = 90;  // Base center
  servoAngles[1] = 160; // Right arm back
  servoAngles[2] = 0;   // Left arm back
  servoAngles[3] = 135; // Gripper closed
  updateServos();
}

/**
 * Convenience function: Extend arm
 */
void extendArm() {
  servoAngles[0] = 90;  // Base center
  servoAngles[1] = 60;  // Right arm forward
  servoAngles[2] = 100; // Left arm forward
  servoAngles[3] = 45;  // Gripper open
  updateServos();
}
