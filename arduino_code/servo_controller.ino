/*
 * Pan/Tilt Servo Controller for Webcam Tracking
 * 
 * This Arduino sketch receives commands from Python over serial
 * and controls two servo motors for pan (horizontal) and tilt (vertical) movement.
 * 
 * Wiring:
 * - Pan Servo:  Signal -> Pin 9,  VCC -> 5V (external), GND -> GND
 * - Tilt Servo: Signal -> Pin 10, VCC -> 5V (external), GND -> GND
 * 
 * IMPORTANT: Use an external 5V power supply for servos!
 *            Don't power servos directly from Arduino - it can damage the board.
 * 
 * Commands (from Python):
 * - "M<pan_delta>,<tilt_delta>\n"  : Move by delta values
 * - "S<pan_angle>,<tilt_angle>\n"  : Set absolute position
 * - "X\n"                          : Stop/hold current position
 * - "C\n"                          : Center both servos
 */

#include <Servo.h>

// Pin definitions
const int PAN_SERVO_PIN = 9;
const int TILT_SERVO_PIN = 10;

// Servo limits (adjust based on your hardware)
const int PAN_MIN = 0;
const int PAN_MAX = 180;
const int PAN_CENTER = 90;

const int TILT_MIN = 30;    // Limit to prevent extreme angles
const int TILT_MAX = 150;
const int TILT_CENTER = 90;

// Movement settings
const float SMOOTHING = 0.3;  // Lower = smoother but slower (0.1 - 1.0)
const int UPDATE_INTERVAL = 20;  // ms between servo updates

// Servo objects
Servo panServo;
Servo tiltServo;

// Current positions
float currentPan = PAN_CENTER;
float currentTilt = TILT_CENTER;

// Target positions
float targetPan = PAN_CENTER;
float targetTilt = TILT_CENTER;

// Timing
unsigned long lastUpdate = 0;

// Serial buffer
String inputBuffer = "";

void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("Pan/Tilt Servo Controller Ready");
  
  // Attach servos
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  
  // Center servos on startup
  centerServos();
  
  Serial.println("Servos centered");
}

void loop() {
  // Read serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
  
  // Update servo positions with smoothing
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    updateServos();
    lastUpdate = currentTime;
  }
}

void processCommand(String cmd) {
  if (cmd.length() < 1) return;
  
  char cmdType = cmd.charAt(0);
  String params = cmd.substring(1);
  
  switch (cmdType) {
    case 'M':  // Move by delta
      handleMoveCommand(params);
      break;
      
    case 'S':  // Set absolute position
      handleSetCommand(params);
      break;
      
    case 'X':  // Stop
      // Hold current position (target = current)
      targetPan = currentPan;
      targetTilt = currentTilt;
      break;
      
    case 'C':  // Center
      centerServos();
      break;
      
    default:
      Serial.println("Unknown command");
      break;
  }
}

void handleMoveCommand(String params) {
  // Parse "pan_delta,tilt_delta"
  int commaIndex = params.indexOf(',');
  if (commaIndex < 0) return;
  
  int panDelta = params.substring(0, commaIndex).toInt();
  int tiltDelta = params.substring(commaIndex + 1).toInt();
  
  // Scale delta values (input is -100 to 100, convert to degrees)
  float panChange = panDelta * 0.05;   // Adjust multiplier for sensitivity
  float tiltChange = tiltDelta * 0.05;
  
  // Update targets
  targetPan = constrain(targetPan + panChange, PAN_MIN, PAN_MAX);
  targetTilt = constrain(targetTilt + tiltChange, TILT_MIN, TILT_MAX);
}

void handleSetCommand(String params) {
  // Parse "pan_angle,tilt_angle"
  int commaIndex = params.indexOf(',');
  if (commaIndex < 0) return;
  
  int panAngle = params.substring(0, commaIndex).toInt();
  int tiltAngle = params.substring(commaIndex + 1).toInt();
  
  // Set targets
  targetPan = constrain(panAngle, PAN_MIN, PAN_MAX);
  targetTilt = constrain(tiltAngle, TILT_MIN, TILT_MAX);
}

void updateServos() {
  // Smooth interpolation towards target
  currentPan += (targetPan - currentPan) * SMOOTHING;
  currentTilt += (targetTilt - currentTilt) * SMOOTHING;
  
  // Write to servos
  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);
}

void centerServos() {
  targetPan = PAN_CENTER;
  targetTilt = TILT_CENTER;
  currentPan = PAN_CENTER;
  currentTilt = TILT_CENTER;
  
  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
}
