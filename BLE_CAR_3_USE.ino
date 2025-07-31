// APP use V.19

#define ENA 10  // Enable/speed motor Right
#define ENB 11  // Enable/speed motor Left

#define IN1 3  // L298N in1 motor Right
#define IN2 5  // L298N in2 motor Right
#define IN3 6  // L298N in3 motor Left
#define IN4 9  // L298N in4 motor Left

const int MAX_SPEED = 255;
const int MIN_SPEED = 0;
const int SPEED_INCREMENT = 10;
const int speed_Coeff = 4;
const unsigned long debounceDelay = 50;

// Motor direction correction flags
bool rightMotorReversed = false;  // Set to true if right motor spins backward
bool leftMotorReversed = false;   // Set to true if left motor spins backward

// Motor speed balancing factors (range: 0.5 - 1.5)
float rightMotorFactor = 1.0;  // Multiplier for right motor speed
float leftMotorFactor = 1.0;   // Multiplier for left motor speed

int command = 0;
int speedCar = 75;
unsigned long lastCommandTime = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);

  // Test motor directions on startup
  Serial.println("Testing motor directions...");
  testMotorDirections();

  Serial.println("Speed balancing commands:");
  Serial.println("6: Decrease right motor speed");
  Serial.println("7: Increase right motor speed");
  Serial.println("8: Decrease left motor speed");
  Serial.println("9: Increase left motor speed");
  Serial.println("0: Reset both motors to 1.0");
}

void testMotorDirections() {
  Serial.println("Testing Right Motor Forward...");
  setRightMotor(true, 100);  // Forward
  delay(1000);
  stopRobot();
  delay(500);

  Serial.println("Testing Left Motor Forward...");
  setLeftMotor(true, 100);  // Forward
  delay(1000);
  stopRobot();
  delay(500);

  Serial.println("Motor test complete. Check directions and update flags if needed.");
}

// Calculate actual speed with balancing factor
int calculateActualSpeed(int baseSpeed, float factor) {
  int actualSpeed = (int)(baseSpeed * factor);
  return constrain(actualSpeed, 0, MAX_SPEED);
}

// Improved motor control functions with direction correction and speed balancing
void setRightMotor(bool forward, int speed) {
  bool actualForward = rightMotorReversed ? !forward : forward;
  int actualSpeed = calculateActualSpeed(speed, rightMotorFactor);

  if (actualForward) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, actualSpeed);
}

void setLeftMotor(bool forward, int speed) {
  bool actualForward = leftMotorReversed ? !forward : forward;
  int actualSpeed = calculateActualSpeed(speed, leftMotorFactor);

  if (actualForward) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, actualSpeed);
}

void sendSpeedToApp() {
  Serial.print("SPEED:");
  Serial.println(speedCar);
}

void sendBalanceFactors() {
  Serial.print("Right motor factor: ");
  Serial.println(rightMotorFactor, 2);
  Serial.print("Left motor factor: ");
  Serial.println(leftMotorFactor, 2);
}

void updateSpeed(int targetSpeed) {
  if (speedCar < targetSpeed) {
    speedCar += SPEED_INCREMENT;
    if (speedCar > targetSpeed) speedCar = targetSpeed;
  } else if (speedCar > targetSpeed) {
    speedCar -= SPEED_INCREMENT;
    if (speedCar < targetSpeed) speedCar = targetSpeed;
  }
  sendSpeedToApp();
}

// Updated movement functions using the new motor control
void goAhead() {
  setRightMotor(true, speedCar);
  setLeftMotor(true, speedCar);
}

void goBack() {
  setRightMotor(false, speedCar);
  setLeftMotor(false, speedCar);
}

void goRight() {
  setRightMotor(true, speedCar);  // Right motor forward
  setLeftMotor(false, speedCar);  // Left motor backward
}

void goLeft() {
  setRightMotor(false, speedCar);  // Right motor backward
  setLeftMotor(true, speedCar);    // Left motor forward
}

void goAheadRight() {
  setRightMotor(true, speedCar / speed_Coeff);  // Slower right
  setLeftMotor(true, speedCar);                 // Full speed left
}

void goAheadLeft() {
  setRightMotor(true, speedCar);               // Full speed right
  setLeftMotor(true, speedCar / speed_Coeff);  // Slower left
}

void goBackRight() {
  setRightMotor(false, speedCar / speed_Coeff);  // Slower right backward
  setLeftMotor(false, speedCar);                 // Full speed left backward
}

void goBackLeft() {
  setRightMotor(false, speedCar);               // Full speed right backward
  setLeftMotor(false, speedCar / speed_Coeff);  // Slower left backward
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// Motor balancing commands
void handleBalancingCommand(char cmd) {
  const float BALANCE_INCREMENT = 0.05;

  switch (cmd) {
    case '6':  // Decrease right motor speed
      rightMotorFactor = max(0.5, rightMotorFactor - BALANCE_INCREMENT);
      Serial.print("Right motor factor decreased to: ");
      Serial.println(rightMotorFactor, 2);
      break;

    case '7':  // Increase right motor speed
      rightMotorFactor = min(1.5, rightMotorFactor + BALANCE_INCREMENT);
      Serial.print("Right motor factor increased to: ");
      Serial.println(rightMotorFactor, 2);
      break;

    case '8':  // Decrease left motor speed
      leftMotorFactor = max(0.5, leftMotorFactor - BALANCE_INCREMENT);
      Serial.print("Left motor factor decreased to: ");
      Serial.println(leftMotorFactor, 2);
      break;

    case '9':  // Increase left motor speed
      leftMotorFactor = min(1.5, leftMotorFactor + BALANCE_INCREMENT);
      Serial.print("Left motor factor increased to: ");
      Serial.println(leftMotorFactor, 2);
      break;

    case '0':  // Reset both motors to 1.0
      rightMotorFactor = 1.0;
      leftMotorFactor = 1.0;
      Serial.println("Both motor factors reset to 1.0");
      sendBalanceFactors();
      break;
  }
}

// Existing calibration commands
void handleCalibrationCommand(char cmd) {
  switch (cmd) {
    case '1':  // Test right motor forward
      Serial.println("Right motor forward test");
      setRightMotor(true, 150);
      delay(1000);
      stopRobot();
      break;

    case '2':  // Test left motor forward
      Serial.println("Left motor forward test");
      setLeftMotor(true, 150);
      delay(1000);
      stopRobot();
      break;

    case '3':  // Toggle right motor direction
      rightMotorReversed = !rightMotorReversed;
      Serial.print("Right motor reversed: ");
      Serial.println(rightMotorReversed ? "YES" : "NO");
      break;

    case '4':  // Toggle left motor direction
      leftMotorReversed = !leftMotorReversed;
      Serial.print("Left motor reversed: ");
      Serial.println(leftMotorReversed ? "YES" : "NO");
      break;

    case '5':  // Show current settings
      Serial.print("Right motor reversed: ");
      Serial.println(rightMotorReversed ? "YES" : "NO");
      Serial.print("Left motor reversed: ");
      Serial.println(leftMotorReversed ? "YES" : "NO");
      sendBalanceFactors();
      break;
  }
}

void handleMovementCommand(char cmd) {
  switch (cmd) {
    case 'F': goAhead(); break;
    case 'B': goBack(); break;
    case 'L': goLeft(); break;
    case 'R': goRight(); break;
    case 'I': goAheadRight(); break;
    case 'G': goAheadLeft(); break;
    case 'J': goBackRight(); break;
    case 'H': goBackLeft(); break;
    case 'S': stopRobot(); break;

    // Calibration commands
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      handleCalibrationCommand(cmd);
      break;

    // Motor balancing commands
    case '6':
    case '7':
    case '8':
    case '9':
    case '0':
      handleBalancingCommand(cmd);
      break;

    default:
      Serial.println("Invalid Command");
      Serial.println("Movement: F,B,L,R,I,G,J,H,S");
      Serial.println("Calibration: 1,2,3,4,5");
      Serial.println("Balance: 6,7,8,9,0");
      break;
  }
}

void handleControlCommand(char cmd) {
  switch (cmd) {
    case '+': updateSpeed(min(speedCar + SPEED_INCREMENT, MAX_SPEED)); break;
    case '-': updateSpeed(max(speedCar - SPEED_INCREMENT, MIN_SPEED)); break;
    default: Serial.println("Invalid Control Command"); break;
  }
}

void loop() {
  if (Serial.available() > 0) {
    unsigned long currentTime = millis();
    if (currentTime - lastCommandTime >= debounceDelay) {
      command = Serial.read();
      if (command == '+' || command == '-') {
        handleControlCommand(command);
      } else {
        handleMovementCommand(command);
      }
      lastCommandTime = currentTime;
    }
  }
}