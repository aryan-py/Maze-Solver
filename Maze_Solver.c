#include <Wire.h>
#include <EEPROM.h>

// MPU6050 constants
const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// Motor control pins
const int leftSpeed = 9;
const int rightSpeed = 5;
const int left1 = 3;
const int left2 = 2;
const int right1 = 8;
const int right2 = 4;

// IR Sensor Pins
const int leftIR = 10;
const int frontIR = 11;
const int rightIR = 12;

// Motor speed constants
const int maxSpeed = 255;
const int minSpeed = 160;
int equilibriumSpeed = 248;

// Movement control variables
float angle;
float targetAngle = 0;
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false;
bool prevIsDriving = true;

// EEPROM Addresses
int startCellAddress = 0;
int goalCellAddress = 2;
int visitedCellsStartAddress = 4;
int visitedCellsIndex = 0;

// Cell size in meters
const float CELL_SIZE = 0.3;

struct Cell {
  int x;
  int y;
};

// Stack for DFS
Cell stack[100];
int stackTop = -1;

Cell currentCell = {0, 0};
Cell goalCell = {4, 4};
bool goalFound = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  initializeMPU6050();
  calculateError();
  
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  
  pinMode(leftIR, INPUT);
  pinMode(frontIR, INPUT);
  pinMode(rightIR, INPUT);
  
  EEPROM.put(startCellAddress, currentCell);
  EEPROM.put(goalCellAddress, goalCell);
  
  currentTime = micros();
}

void loop() {
  if (!goalFound) {
    depthFirstSearch();
  } else {
    Serial.println("Goal reached!");
    stopCar();
    while (1);
  }
}

void depthFirstSearch() {
  if (isGoal(currentCell)) {
    goalFound = true;
    return;
  }

  storeVisitedCell(currentCell);

  if (canMoveForward()) {
    moveForward();
    pushToStack(currentCell);
  } else if (canTurnLeft()) {
    turnLeft();
    pushToStack(currentCell);
  } else if (canTurnRight()) {
    turnRight();
    pushToStack(currentCell);
  } else if (stackTop >= 0) {
    turnAround();
    currentCell = popFromStack();
  }
}

bool isGoal(Cell cell) {
  return (cell.x == goalCell.x && cell.y == goalCell.y);
}

void storeVisitedCell(Cell cell) {
  EEPROM.put(visitedCellsStartAddress + visitedCellsIndex * sizeof(Cell), cell);
  visitedCellsIndex++;
}

bool canMoveForward() {
  return digitalRead(frontIR) == LOW;
}

bool canTurnLeft() {
  return digitalRead(leftIR) == LOW;
}

bool canTurnRight() {
  return digitalRead(rightIR) == LOW;
}

void moveForward() {
  targetAngle = angle;
  isDriving = true;
  forward();
  float startAngle = angle;
  float distanceMoved = 0;
  
  while (distanceMoved < CELL_SIZE) {
    updateMPU6050Data();
    driving();
    
    // Estimate distance moved
    distanceMoved += abs(cos(angle * PI / 180) * (angle - startAngle) * PI / 180 * CELL_SIZE / 90);
    
    if (digitalRead(frontIR) == HIGH) {
      break;
    }
  }
  
  stopCar();
  isDriving = false;
  updateCurrentCell();
}

void turnLeft() {
  targetAngle += 90;
  if (targetAngle > 180) {
    targetAngle -= 360;
  }
  rotate();
  updateCurrentCell();
}

void turnRight() {
  targetAngle -= 90;
  if (targetAngle <= -180) {
    targetAngle += 360;
  }
  rotate();
  updateCurrentCell();
}

void turnAround() {
  targetAngle += 180;
  if (targetAngle > 180) {
    targetAngle -= 360;
  }
  rotate();
  updateCurrentCell();
}

void updateCurrentCell() {
  float angleDiff = targetAngle - angle;
  if (abs(angleDiff) < 45) {
    currentCell.x++;
  } else if (abs(angleDiff) > 135) {
    currentCell.x--;
  } else if (angleDiff > 0) {
    currentCell.y--;
  } else {
    currentCell.y++;
  }
}

void pushToStack(Cell cell) {
  stack[++stackTop] = cell;
}

Cell popFromStack() {
  return stack[stackTop--];
}

void driving() {
  int deltaAngle = round(targetAngle - angle);
  forward();
  if (deltaAngle != 0) {
    controlSpeed();
    rightSpeedVal = maxSpeed;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}

void controlSpeed() {
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  
  if (deltaAngle > 30) {
    targetGyroX = 60;
  } else if (deltaAngle < -30) {
    targetGyroX = -60;
  } else {
    targetGyroX = 2 * deltaAngle;
  }
  
  if (round(targetGyroX - GyroX) == 0) {
    return;
  } else if (targetGyroX > GyroX) {
    leftSpeedVal = changeSpeed(leftSpeedVal, -1);
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate() {
  isDriving = false;
  float startAngle = angle;
  float angleTurned = 0;
  
  while (abs(angleTurned) < 90) {
    updateMPU6050Data();
    int deltaAngle = round(targetAngle - angle);
    
    if (deltaAngle > 0) {
      left();
    } else {
      right();
    }

    int targetGyroX = (abs(deltaAngle) > 30) ? 60 : 2 * abs(deltaAngle);
    
    if (targetGyroX > abs(GyroX)) {
      leftSpeedVal = changeSpeed(leftSpeedVal, +1);
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
    
    angleTurned = angle - startAngle;
    if (angleTurned > 180) angleTurned -= 360;
    if (angleTurned < -180) angleTurned += 360;
  }
  
  stopCar();
}

int changeSpeed(int motorSpeed, int increment) {
  motorSpeed += increment;
  if (motorSpeed > maxSpeed) {
    motorSpeed = maxSpeed;
  } else if (motorSpeed < minSpeed) {
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

void updateMPU6050Data() {
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0;
  
  readAcceleration();
  readGyro();
  
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = roll;
}

void initializeMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void calculateError() {
  // ... (Keep the existing calculateError function)
}

void readAcceleration() {
  // ... (Keep the existing readAcceleration function)
}

void readGyro() {
  // ... (Keep the existing readGyro function)
}

void stopCar() {
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

void forward() {
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void left() {
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void right() {
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
}
