// Line follower robot with V-shaped sensor arrangement
// Inner sensors are 6cm apart and positioned 3cm ahead of outer sensors
// Outer sensors are still at original width at 16cm apart
// Sensors on each side (left/right) are placed in a V formation
// Lane is 20cm wide with two black lines 20cm apart and 2cm wide each
// The robot is 16cm wide
// lane consists of 30 turns, 90 turns

// Motor control pins
int IN1 = 2;
int IN2 = 4;
int IN3 = 7;
int IN4 = 8;
int ENA = 3;
int ENB = 5;

// Sensor pins and variables
int Lsensor1 = A1;  // Left outer sensor
int Lsensor2 = A3;  // Left inner sensor
int Rsensor1 = A0;  // Right outer sensor
int Rsensor2 = A2;  // Right inner sensor
int RIGHT1, RIGHT2, LEFT1, LEFT2;
int thresholdValue = 200;  // Adjusted based on readings (white: 46-50, black: 800)

// Motor speeds
int baseSpeed = 70;  // Reduced from 60 to allow for more precise control
int innerTurnAdjust = 8;   // Reduced from 10 to prevent overshoot
int outerTurnAdjust = 5;  // Subtract from base speed for outer sensor corrections (gentler)
int turn90Adjust = 12;     // Reduced from 15 to prevent overshooting

// Motor balance calibration (kept from original)
float leftMotorFactor = 0.85;  // Left motor power factor
float rightMotorFactor = 1.20;  // Right motor power factor

// PID Controller Variables
float Kp = 1.0;    // Reduced from 1.2 to make turns less aggressive
float Ki = 0.003;  // Reduced from 0.005 to minimize overshoot
float Kd = 2.0;    // Increased from 1.8 for better damping of oscillations
float error = 0;   // Current error
float lastError = 0; // Previous error
float integral = 0; // Integral of error
float derivative = 0; // Derivative of error
int pidOutput = 0; // PID controller output

// Turn detection flags
bool is90DegreeTurn = false;
bool needsSharpTurn = false;

// Extra variable to smooth transitions
int transitionCounter = 0;
const int transitionPeriod = 8; // Increased from 5 for smoother transition

// Action state tracking
#define STATE_IDLE 0
#define STATE_FORWARD 1
#define STATE_SLOW_FORWARD 2
#define STATE_TURNING_LEFT_INNER 3
#define STATE_TURNING_LEFT_OUTER 4
#define STATE_TURNING_RIGHT_INNER 5
#define STATE_TURNING_RIGHT_OUTER 6
#define STATE_STOPPED 7
#define STATE_PID_CONTROL 8

int currentState = STATE_IDLE;
int previousState = STATE_IDLE;

// Turn duration control
const unsigned long turnDuration = 250;     // Reduced from 300 to prevent overshooting
const unsigned long turn90Duration = 800;   // Reduced from 500 to prevent overshooting
unsigned long turnStartTime = 0;
bool isTurning = false;
int currentTurnDirection = 0;  // 0 = none, 1 = left, 2 = right

// Define LED pin if available
const int LED_PIN = 13; // Built-in LED on most Arduinos

// LED blink timing
unsigned long lastLedBlinkTime = 0;
const int ledBlinkInterval = 150; // Time in ms for LED blink
bool ledState = false;
int blinkCount = 0;
int totalBlinks = 0;

// Helper functions for precise rotation
void rotateLeft(int speed) {
  int leftSpeed = speed * leftMotorFactor;
  int rightSpeed = speed * rightMotorFactor;
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  digitalWrite(IN1, LOW);  // Left motor reverse
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void rotateRight(int speed) {
  int leftSpeed = speed * leftMotorFactor;
  int rightSpeed = speed * rightMotorFactor;
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  digitalWrite(IN1, HIGH); // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Right motor reverse
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

// PID control function
void calculatePID() {
  // Calculate error based on sensor values
  // Positive error: robot is to the right of line (need to turn left)
  // Negative error: robot is to the left of line (need to turn right)
  error = 0;
  
  // Special case detection for sharp turns
  is90DegreeTurn = false;
  needsSharpTurn = false;
  
  // Modified 90-degree turn detection
  // Both inner sensors detect black (V-formation ahead of outer sensors)
  if ((LEFT2 && RIGHT2 && !LEFT1 && !RIGHT1)) {
    is90DegreeTurn = true;
    // Determine turn direction based on previous error direction
    error = (lastError >= 0) ? 5 : -5;  // Maintain turn direction consistency
    needsSharpTurn = true;
  }
  // Both left sensors on black - sharp right turn needed
  else if (LEFT1 && LEFT2 && !RIGHT1 && !RIGHT2) {
    is90DegreeTurn = true;
    error = -5; // Reduced from -6 to prevent overshoot
    needsSharpTurn = true;
  } 
  // Both right sensors on black - sharp left turn needed
  else if (RIGHT1 && RIGHT2 && !LEFT1 && !LEFT2) {
    is90DegreeTurn = true;
    error = 5; // Reduced from 6 to prevent overshoot
    needsSharpTurn = true;
  }
  // Inner sensors detect turn before outer sensors
  else if (LEFT2 && !LEFT1 && !RIGHT1 && !RIGHT2) {
    // Only inner left sensor - approaching right turn
    error = -4; // Stronger response due to forward position
    needsSharpTurn = true;
  }
  else if (RIGHT2 && !RIGHT1 && !LEFT1 && !LEFT2) {
    // Only inner right sensor - approaching left turn
    error = 4; // Stronger response due to forward position
    needsSharpTurn = true;
  }
  // Regular error calculation for normal line following
  else {
    // Weight sensors based on position and V-arrangement
    // Outer sensors have higher weight due to their position
    if (LEFT1) error -= 4;  // Increased from 3 due to importance of outer sensors
    if (LEFT2) error -= 1;  // Inner sensor (forward position)
    if (RIGHT2) error += 1; // Inner sensor (forward position)
    if (RIGHT1) error += 4; // Increased from 3 due to importance of outer sensors
    
    // Check if a stronger turn is needed for single sensor cases
    if ((LEFT1 && !LEFT2 && !RIGHT1 && !RIGHT2) || 
        (RIGHT1 && !RIGHT2 && !LEFT1 && !LEFT2)) {
      needsSharpTurn = true;
    }
  }
  
  // Calculate derivative first (before modifying error)
  derivative = error - lastError;
  
  // Apply a lowpass filter to error to reduce noise and oscillation
  error = error * 0.65 + lastError * 0.35; // More smoothing (was 0.7/0.3)
  
  // PID calculation
  integral = integral + error;
  
  // Reset integral when error changes sign (crossed the center)
  if ((error * lastError) < 0) {
    integral = integral * 0.5; // Reduce integral when crossing center
  }
  
  // Limit integral to prevent wind-up
  integral = constrain(integral, -30, 30);
  
  // Calculate PID output with possibly enhanced gains for turns
  if (is90DegreeTurn) {
    // For 90-degree turns, reduced gains slightly
    pidOutput = (Kp * 1.15 * error) + (Ki * 0.8 * integral) + (Kd * 1.6 * derivative);
  } else if (needsSharpTurn) {
    // For approaching turns detected by inner sensors (V-arrangement benefit)
    pidOutput = (Kp * error) + (Ki * 0.9 * integral) + (Kd * 1.4 * derivative);
  } else {
    pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  }
  
  // Limit maximum PID output to prevent overshoot
  pidOutput = constrain(pidOutput, -50, 50);
  
  // Update last error
  lastError = error;
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(Lsensor1, INPUT);
  pinMode(Lsensor2, INPUT);
  pinMode(Rsensor1, INPUT);
  pinMode(Rsensor2, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initial stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  Serial.println("Line Follower Starting...");
  
  // Signal startup with LED
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  // Initial state message
  printRobotStatus();
  
  // Wait 5 seconds before starting, with LED indication
  Serial.println("Starting line following in 5 seconds...");
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    Serial.print(".");
  }
  Serial.println("GO!");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors and convert to digital (1 = black line, 0 = white)
  int rightVal1 = analogRead(Rsensor1);  // Outer right
  int rightVal2 = analogRead(Rsensor2);  // Inner right
  int leftVal1 = analogRead(Lsensor1);   // Outer left
  int leftVal2 = analogRead(Lsensor2);   // Inner left
  
  RIGHT1 = (rightVal1 > thresholdValue) ? 1 : 0;
  RIGHT2 = (rightVal2 > thresholdValue) ? 1 : 0;
  LEFT1 = (leftVal1 > thresholdValue) ? 1 : 0;
  LEFT2 = (leftVal2 > thresholdValue) ? 1 : 0;

  // Save previous state before updating
  previousState = currentState;

  // Special cases that override PID control

  // 1. All sensors detect black (both lines) - STOP
  if (RIGHT1 && RIGHT2 && LEFT1 && LEFT2) {
    stopMotors();
    currentState = STATE_STOPPED;
    if (previousState != currentState) {
      printRobotStatus();
    }
    return;
  }
  
  // 2. Check for 3 sensors black - STOP
  if ((LEFT1 && LEFT2 && RIGHT2) || (LEFT1 && LEFT2 && RIGHT1) || 
      (RIGHT1 && RIGHT2 && LEFT2) || (RIGHT1 && RIGHT2 && LEFT1)) {
    stopMotors();
    currentState = STATE_STOPPED;
    if (previousState != currentState) {
      printRobotStatus();
    }
    return;
  }
  
  // 3. All sensors detect white (between lines) - MOVE FORWARD
  if (!RIGHT1 && !RIGHT2 && !LEFT1 && !LEFT2) {
    // If we just came from a turn state, gradual transition to normal
    if (currentState == STATE_TURNING_LEFT_INNER || currentState == STATE_TURNING_RIGHT_INNER ||
        currentState == STATE_TURNING_LEFT_OUTER || currentState == STATE_TURNING_RIGHT_OUTER) {
      
      // Check if we need to continue a smooth transition
      if (transitionCounter < transitionPeriod) {
        transitionCounter++;
        // Gradually reduce turn effect with a more gradual curve
        float transitionFactor = 1.0 - pow((float)transitionCounter / transitionPeriod, 0.7);
        
        if (currentTurnDirection == 1) {
          // Left turn transition - strengthen right motor, weaken left motor
          int leftSpeed = baseSpeed * (0.7 + 0.3 * (1.0 - transitionFactor));
          int rightSpeed = baseSpeed;
          moveForward(leftSpeed, rightSpeed);
        } else if (currentTurnDirection == 2) {
          // Right turn transition - strengthen left motor, weaken right motor
          int leftSpeed = baseSpeed;
          int rightSpeed = baseSpeed * (0.7 + 0.3 * (1.0 - transitionFactor));
          moveForward(leftSpeed, rightSpeed);
        } else {
          moveForward(baseSpeed, baseSpeed);
          currentState = STATE_FORWARD;
          integral = 0; // Reset integral when on straight line
        }
      } else {
        // Transition complete
        moveForward(baseSpeed, baseSpeed);
        currentState = STATE_FORWARD;
        integral = 0; // Reset integral when on straight line
        transitionCounter = 0;
      }
    } else {
    moveForward(baseSpeed, baseSpeed);
    currentState = STATE_FORWARD;
      integral = 0; // Reset integral when on straight line
      transitionCounter = 0;
    }
    
    if (previousState != currentState) {
      printRobotStatus();
    }
    return;
  } else {
    // Reset transition counter when not all white
    transitionCounter = 0;
  }
  
  // PID CONTROL for all other cases
  calculatePID();
  currentState = STATE_PID_CONTROL;
  
  // Apply PID output to motor speeds with smoother ratio
  // This prevents one motor stopping completely while the other is at full power
  int leftSpeed, rightSpeed;
  
  if (pidOutput > 0) { // Need to turn left
    leftSpeed = baseSpeed - constrain(pidOutput, 0, baseSpeed - 10);
    rightSpeed = baseSpeed + constrain(pidOutput/2, 0, 30);
  } else { // Need to turn right
    leftSpeed = baseSpeed + constrain(-pidOutput/2, 0, 30);
    rightSpeed = baseSpeed - constrain(-pidOutput, 0, baseSpeed - 10);
  }
  
  // Set turn direction based on error
  if (pidOutput > 0) {
    currentTurnDirection = 1; // Left turn
    if (is90DegreeTurn) {
      currentState = STATE_TURNING_LEFT_INNER;
      isTurning = true;
      turnStartTime = currentTime;
    } else if (needsSharpTurn) {
      currentState = STATE_TURNING_LEFT_OUTER;
      isTurning = true;
      turnStartTime = currentTime;
    }
  } else if (pidOutput < 0) {
    currentTurnDirection = 2; // Right turn
    if (is90DegreeTurn) {
      currentState = STATE_TURNING_RIGHT_INNER;
      isTurning = true;
      turnStartTime = currentTime;
    } else if (needsSharpTurn) {
      currentState = STATE_TURNING_RIGHT_OUTER;
      isTurning = true;
      turnStartTime = currentTime;
    }
  } else {
    currentTurnDirection = 0;
    isTurning = false;
  }
  
  // Special handling for turns
  if (isTurning) {
    // Get the appropriate turn duration for current state
    unsigned long currentTurnDuration = turnDuration;
    if (is90DegreeTurn) {
      currentTurnDuration = turn90Duration;
      digitalWrite(LED_PIN, HIGH); // Visual indication of 90-degree turn
    }
    
    // Continue turning until the duration is over
    if (currentTime - turnStartTime >= currentTurnDuration) {
      isTurning = false;
      digitalWrite(LED_PIN, LOW);
      
      // Start with a slow exit from turn
      int slowExitSpeed = baseSpeed * 0.7;
      if (currentTurnDirection == 1) {
        moveForward(slowExitSpeed, baseSpeed);
      } else if (currentTurnDirection == 2) {
        moveForward(baseSpeed, slowExitSpeed);
      }
    } else {
      // Continue the turn with appropriate speed, but start slowing down near the end
      float completionRatio = (float)(currentTime - turnStartTime) / currentTurnDuration;
      int turnSpeed;
      
      if (completionRatio > 0.7) {
        // Start slowing down in the last 30% of the turn
        float slowdownFactor = 1.0 - ((completionRatio - 0.7) / 0.3) * 0.3; // Gradually reduce to 70% speed
        turnSpeed = is90DegreeTurn ? 
                   (baseSpeed + turn90Adjust) * slowdownFactor : 
                   (baseSpeed + innerTurnAdjust) * slowdownFactor;
      } else {
        turnSpeed = is90DegreeTurn ? 
                   baseSpeed + turn90Adjust : 
                   baseSpeed + innerTurnAdjust;
      }
      
      if (currentTurnDirection == 1) {
        rotateLeft(turnSpeed);
      } else if (currentTurnDirection == 2) {
        rotateRight(turnSpeed);
      }
      
      // Print state if changed
      if (previousState != currentState) {
        printRobotStatus();
      }
      return; // Skip the rest of the loop while turning
    }
  }
  
  // Ensure speeds are within valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // If error is significant, use differential drive, otherwise use forward motion
  if (abs(error) >= 2 && !isTurning) {
    // For significant errors, use rotation but with smoother approach
    if (pidOutput > 0) {
      int turnSpeed = min(abs(pidOutput) + baseSpeed/3, baseSpeed + 30);
      rotateLeft(turnSpeed);
    } else {
      int turnSpeed = min(abs(pidOutput) + baseSpeed/3, baseSpeed + 30);
      rotateRight(turnSpeed);
    }
  } else if (!isTurning) {
    // For small errors, adjust forward speeds
    moveForward(leftSpeed, rightSpeed);
  }
  
  // Print state if changed
  if (previousState != currentState) {
    printRobotStatus();
  }
  
  delay(2); // Ultra-fast update rate (500Hz)
}

// Function to print the current robot status when state changes
void printRobotStatus() {
  // Read sensors
  int rightVal1 = analogRead(Rsensor1);
  int rightVal2 = analogRead(Rsensor2);
  int leftVal1 = analogRead(Lsensor1);
  int leftVal2 = analogRead(Lsensor2);
  
  // Print a cleaner header
  Serial.println("\n========== ROBOT STATUS ==========");
  
  // Print timestamp
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.println(" ms");
  
  // Print sensor readings in a compact, readable format
  Serial.println("SENSORS (RAW | STATE):");
  Serial.print("  Left:  ");
  Serial.print(leftVal1);
  Serial.print(" | ");
  Serial.print((leftVal1 > thresholdValue) ? "BLACK" : "white");
  Serial.print("  |  ");
  Serial.print(leftVal2);
  Serial.print(" | ");
  Serial.println((leftVal2 > thresholdValue) ? "BLACK" : "white");
  
  Serial.print("  Right: ");
  Serial.print(rightVal2);
  Serial.print(" | ");
  Serial.print((rightVal2 > thresholdValue) ? "BLACK" : "white");
  Serial.print("  |  ");
  Serial.print(rightVal1);
  Serial.print(" | ");
  Serial.println((rightVal1 > thresholdValue) ? "BLACK" : "white");
  
  // Print current action with clearer formatting
  Serial.print("NEW ACTION: ");
  
  // Print the appropriate action based on the current state
  switch (currentState) {
    case STATE_IDLE:
      Serial.println("[IDLE]");
      break;
    case STATE_FORWARD:
      Serial.println("[FORWARD] - Between lines");
      break;
    case STATE_SLOW_FORWARD:
      Serial.println("[SLOW FORWARD] - Default mode");
      break;
    case STATE_TURNING_LEFT_INNER:
      Serial.println("[STRONG LEFT TURN] - Inner right sensor or both right sensors on black");
      break;
    case STATE_TURNING_LEFT_OUTER:
      Serial.println("[GENTLE LEFT TURN] - Outer right sensor on black");
      break;
    case STATE_TURNING_RIGHT_INNER:
      Serial.println("[STRONG RIGHT TURN] - Inner left sensor or both left sensors on black");
      break;
    case STATE_TURNING_RIGHT_OUTER:
      Serial.println("[GENTLE RIGHT TURN] - Outer left sensor on black");
      break;
    case STATE_STOPPED:
      Serial.println("[STOPPED] - All sensors on black or 3 sensors on black");
      break;
    case STATE_PID_CONTROL:
      Serial.print("[PID CONTROL] - Error: ");
      Serial.print(error);
      Serial.print(", PID Output: ");
      Serial.print(pidOutput);
      if (is90DegreeTurn) {
        Serial.print(" (90° TURN)");
      } else if (needsSharpTurn) {
        Serial.print(" (SHARP TURN)");
      }
      Serial.println();
      break;
    default:
      Serial.println("[UNKNOWN STATE]");
  }
  
  if (currentState == STATE_PID_CONTROL) {
    Serial.print("PID Values - P: ");
    Serial.print(error * Kp);
    Serial.print(", I: ");
    Serial.print(integral * Ki);
    Serial.print(", D: ");
    Serial.print(derivative * Kd);
    if (is90DegreeTurn) {
      Serial.print(" - Using 90° turn mode");
    }
    Serial.println();
  }
  
  Serial.println("====================================");
}

// HELPER FUNCTIONS
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveForward(int leftSpeed, int rightSpeed) {
  // Apply motor factors
  leftSpeed = leftSpeed * leftMotorFactor;
  rightSpeed = rightSpeed * rightMotorFactor;
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  digitalWrite(IN1, HIGH); // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Right motor forward
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}