// Line follower robot with simplified logic for four sensor arrangement
// All sensors are positioned between the two black lines
// Outer sensors are 16cm apart, inner sensors are 10cm apart
// Sensors on each side (left/right) are placed next to each other

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
int baseSpeed = 70;  // Base speed for straight line
int innerTurnAdjust = 10;   // Add to base speed for inner sensor corrections (stronger)
int outerTurnAdjust = -20;  // Subtract from base speed for outer sensor corrections (gentler)
int turn90Adjust = 20;      // Add to base speed for 90-degree turns (fastest)

// Motor balance calibration (kept from original)
float leftMotorFactor = 0.85;  // Left motor power factor
float rightMotorFactor = 1.20;  // Right motor power factor

// Action state tracking
#define STATE_IDLE 0
#define STATE_FORWARD 1
#define STATE_SLOW_FORWARD 2
#define STATE_TURNING_LEFT_INNER 3
#define STATE_TURNING_LEFT_OUTER 4
#define STATE_TURNING_RIGHT_INNER 5
#define STATE_TURNING_RIGHT_OUTER 6
#define STATE_STOPPED 7

int currentState = STATE_IDLE;
int previousState = STATE_IDLE;

// Turn duration control
const unsigned long turnDuration = 400;     // Regular turn duration in milliseconds (reduced)
const unsigned long turn90Duration = 800;   // 90-degree turn needs more time
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

  // Check if we're currently in a turning state
  if (isTurning) {
    // Set the current state based on turn direction and sensor
    if (currentTurnDirection == 1) { // Left turn
      // Keep existing state if it's already a turning state
      if (currentState == STATE_TURNING_LEFT_INNER || 
          currentState == STATE_TURNING_LEFT_OUTER) {
        // Keep state as is
      } else {
        // Default to inner if state is not already set correctly
        currentState = STATE_TURNING_LEFT_INNER;
      }
    } else if (currentTurnDirection == 2) { // Right turn
      if (currentState == STATE_TURNING_RIGHT_INNER || 
          currentState == STATE_TURNING_RIGHT_OUTER) {
        // Keep state as is
      } else {
        // Default to inner if state is not already set correctly
        currentState = STATE_TURNING_RIGHT_INNER;
      }
    }
    
    // Get the appropriate turn duration for current state
    unsigned long currentTurnDuration = turnDuration;
    if (currentState == STATE_TURNING_LEFT_OUTER || currentState == STATE_TURNING_RIGHT_OUTER) {
      currentTurnDuration = turnDuration;
      totalBlinks = 1; // 1 blink for gentle turn
    } else if (currentState == STATE_TURNING_LEFT_INNER || currentState == STATE_TURNING_RIGHT_INNER) {
      // Check if this is a 90-degree turn (both sensors on one side)
      if ((currentState == STATE_TURNING_LEFT_INNER && (RIGHT1 && RIGHT2)) || 
          (currentState == STATE_TURNING_RIGHT_INNER && (LEFT1 && LEFT2))) {
        currentTurnDuration = turn90Duration; // Use longer duration for 90-degree turns
        totalBlinks = 3; // 3 blinks for 90-degree turn
      } else {
        currentTurnDuration = turnDuration;
        totalBlinks = 2; // 2 blinks for regular stronger turn
      }
    }
    
    // Blink LED based on turn type
    if (currentTime - lastLedBlinkTime >= ledBlinkInterval) {
      lastLedBlinkTime = currentTime;
      
      if (blinkCount < totalBlinks * 2) { // *2 because each blink is ON then OFF
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        blinkCount++;
      } else if (blinkCount >= totalBlinks * 2 && blinkCount < totalBlinks * 2 + 6) {
        // Pause between blink sequences (3 OFF cycles)
        digitalWrite(LED_PIN, LOW);
        blinkCount++;
      } else {
        // Reset blink count to repeat the pattern
        blinkCount = 0;
      }
    }
    
    // Continue turning until the duration is over
    if (currentTime - turnStartTime >= currentTurnDuration) {
      isTurning = false;
      // Turn off LED when turn completes
      digitalWrite(LED_PIN, LOW);
      ledState = false;
      blinkCount = 0;
      // State will be updated in the next loop based on sensor readings
    } else {
      // Continue the turn that was started with the appropriate speed
      int turnSpeed = baseSpeed;
      
      if (currentState == STATE_TURNING_LEFT_INNER || currentState == STATE_TURNING_RIGHT_INNER) {
        turnSpeed = baseSpeed + innerTurnAdjust;
      } else if (currentState == STATE_TURNING_LEFT_OUTER || currentState == STATE_TURNING_RIGHT_OUTER) {
        turnSpeed = baseSpeed + outerTurnAdjust;
      }
      
      // CRITICAL: Ignore sensor inputs during turns to prevent conflicting signals
      if (currentTurnDirection == 1) {
        rotateLeft(turnSpeed);
      } else if (currentTurnDirection == 2) {
        rotateRight(turnSpeed);
      }
      
      // Print state change if needed
      if (previousState != currentState) {
        printRobotStatus();
      } else if (isTurning && (currentTime - turnStartTime) % 250 == 0) {
        // Update turn progress every 250ms
        Serial.print("Still turning: ");
        Serial.print(currentTime - turnStartTime);
        Serial.print("/");
        Serial.print(currentTurnDuration);
        Serial.println(" ms");
      }
      
      return;  // Skip the rest of the loop while turning
    }
  } else {
    // If not turning, ensure LED is off
    digitalWrite(LED_PIN, LOW);
    ledState = false;
    blinkCount = 0;
  }

  // SIMPLIFIED LOGIC:
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
    moveForward(baseSpeed, baseSpeed);
    currentState = STATE_FORWARD;
    if (previousState != currentState) {
      printRobotStatus();
    }
    return;
  }
  
  // 4. Left sensors detect black - TURN RIGHT
  // First check if both left sensors detect black (stronger correction for 90-degree turns)
  if (LEFT1 && LEFT2) {
    // Start a timed right turn with inner speed (for 90-degree turns)
    if (!isTurning) {
      isTurning = true;
      currentTurnDirection = 2;  // Right turn
      turnStartTime = currentTime;
      currentState = STATE_TURNING_RIGHT_INNER;
      if (previousState != currentState) {
        printRobotStatus();
      }
    }
    rotateRight(baseSpeed + innerTurnAdjust);
    return;
  }
  // Then check if only outer left sensor detects black (gentle correction)
  else if (LEFT1) {
    // Start a timed right turn with outer speed
    if (!isTurning) {
      isTurning = true;
      currentTurnDirection = 2;  // Right turn
      turnStartTime = currentTime;
      currentState = STATE_TURNING_RIGHT_OUTER;
      if (previousState != currentState) {
        printRobotStatus();
      }
    }
    rotateRight(baseSpeed + outerTurnAdjust);
    return;
  } 
  // Finally check if only inner left sensor detects black (strong correction)
  else if (LEFT2) {
    // Start a timed right turn with inner speed
    if (!isTurning) {
      isTurning = true;
      currentTurnDirection = 2;  // Right turn
      turnStartTime = currentTime;
      currentState = STATE_TURNING_RIGHT_INNER;
      if (previousState != currentState) {
        printRobotStatus();
      }
    }
    rotateRight(baseSpeed + innerTurnAdjust);
    return;
  }
  
  // 5. Right sensors detect black - TURN LEFT
  // First check if both right sensors detect black (stronger correction for 90-degree turns)
  if (RIGHT1 && RIGHT2) {
    // Start a timed left turn with inner speed (for 90-degree turns)
    if (!isTurning) {
      isTurning = true;
      currentTurnDirection = 1;  // Left turn
      turnStartTime = currentTime;
      currentState = STATE_TURNING_LEFT_INNER;
      if (previousState != currentState) {
        printRobotStatus();
      }
    }
    rotateLeft(baseSpeed + innerTurnAdjust);
    return;
  }
  // Then check if only outer right sensor detects black (gentle correction)
  else if (RIGHT1) {
    // Start a timed left turn with outer speed
    if (!isTurning) {
      isTurning = true;
      currentTurnDirection = 1;  // Left turn
      turnStartTime = currentTime;
      currentState = STATE_TURNING_LEFT_OUTER;
      if (previousState != currentState) {
        printRobotStatus();
      }
    }
    rotateLeft(baseSpeed + outerTurnAdjust);
    return;
  }
  // Finally check if only inner right sensor detects black (strong correction)
  else if (RIGHT2) {
    // Start a timed left turn with inner speed
    if (!isTurning) {
      isTurning = true;
      currentTurnDirection = 1;  // Left turn
      turnStartTime = currentTime;
      currentState = STATE_TURNING_LEFT_INNER;
      if (previousState != currentState) {
        printRobotStatus();
      }
    }
    rotateLeft(baseSpeed + innerTurnAdjust);
    return;
  }
  
  // Default fallback - move forward slowly
  moveForward(baseSpeed/2, baseSpeed/2);
  currentState = STATE_SLOW_FORWARD;
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
    default:
      Serial.println("[UNKNOWN STATE]");
  }
  
  if (isTurning) {
    Serial.print("Turn duration: ");
    Serial.print(turnDuration);
    Serial.print(" ms, Speed adjustment: ");
    
    if (currentState == STATE_TURNING_LEFT_INNER || currentState == STATE_TURNING_RIGHT_INNER) {
      Serial.print("+");
      Serial.print(innerTurnAdjust);
      Serial.println(" (FASTER)"); // Stronger
    } else if (currentState == STATE_TURNING_LEFT_OUTER || currentState == STATE_TURNING_RIGHT_OUTER) {
      Serial.print(outerTurnAdjust);  // This is already negative
      Serial.println(" (SLOWER)"); // Gentler
    } else {
      Serial.println("+0");
    }
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