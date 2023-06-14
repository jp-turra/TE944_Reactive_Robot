// Sound Speed
#define SOUND_SPEED 0.0343 // cm/us

// Ultrassonic Sensors Pins
#define ultrasonFrontTrgPin 30  
#define ultrasonFrontEchoPin 31  

#define ultrasonLeftTrgPin 3 //13  
#define ultrasonLeftEchoPin 4 //12  

#define ultrasonRightTrgPin 13 //3  
#define ultrasonRightEchoPin 12 //4

// Light Sensor Pins
#define lsBackLeftPin A2
#define lsBackRightPin A0
#define lsFrontLeftPin A15
#define lsFrontRightPin A14

// L298N Driver Pins
#define pwm_right_pin 38
#define pwm_left_pin 48

#define in_a_pin 46
#define in_b_pin 44
#define in_c_pin 42
#define in_d_pin 40

// Ultrassonic Limits
#define FRONT_US_LIMIT 20
#define LEFT_US_LIMIT 10
#define RIGHT_US_LIMIT 10

// Vehicle state
enum operationState {
  STOP = 0,
  GO_AHEAD = 1,
  SET_TURN = 2,
  CHECK_TURN = 3
};

operationState state = STOP;

// Ultrassonic Sensors Handlers
float frontUSSensor = 0;
float leftUSSensor = 0;
float rightUSSensor = 0;

// Light Sensor Handlers
float frontLeftLS = 0.0;
float frontRighttLS = 0.0;
float backLeftLS = 0.0;
float backRighttLS = 0.0;

// Motor Speed
float leftSpeed = 0.0;
float rightSpeed = 0.0;

// Pre-call functions
float readUSSensor(int trigger, int echo);
float readLSSensor(int pin);

bool checkLight();
bool hasFrontLight();
bool hasLeftLight();

void setSpeed();

void setup() {
  // Init Serial Comm.
  Serial.begin(115200);

  // Init L298N Module pins
  pinMode(pwm_right_pin, OUTPUT);
  pinMode(in_a_pin, OUTPUT);
  pinMode(in_b_pin, OUTPUT);
  pinMode(in_c_pin, OUTPUT);
  pinMode(in_d_pin, OUTPUT);
  pinMode(pwm_left_pin, OUTPUT);

  // Ultrassonic Pins
  pinMode(ultrasonFrontTrgPin, OUTPUT);
  pinMode(ultrasonLeftTrgPin, OUTPUT);
  pinMode(ultrasonRightTrgPin, OUTPUT);

  pinMode(ultrasonFrontEchoPin, INPUT);
  pinMode(ultrasonLeftEchoPin, INPUT);
  pinMode(ultrasonRightEchoPin, INPUT);

  // Light Sensor pins
  pinMode(lsBackLeftPin, INPUT);
  pinMode(lsBackRighttPin, INPUT);
  pinMode(lsFrontLeftPin, INPUT);
  pinMode(lsFrontRightPin, INPUT);

}

void loop() {
  // Update ultrassonic sensors value
  frontUSSensor = readUSSensor(ultrasonFrontTrgPin, ultrasonFrontEchoPin);
  leftUSSensor = readUSSensor(ultrasonLeftTrgPin, ultrasonLeftEchoPin);
  rightUSSensor = readUSSensor(ultrasonRightTrgPin, ultrasonRightEchoPin);

  // Update Light Sensors Value
  frontLeftLS = readLSSensor(lsFrontLeftPin);
  frontRightLS = readLSSensor(lsFrontRightPin);
  backLeftLS = readLSSensor(lsBackLeftPin);
  backRightLS = readLSSensor(lsBackRightPin);

  // Serial.print("Front: ");
  // Serial.print(frontUSSensor);
  // Serial.print("\tLeft: ");
  // Serial.print(leftUSSensor);
  // Serial.print("\tRight: ");
  // Serial.println(rightUSSensor);

  if (!checkLight()) {
    break;
  }

  if (state == STOP) {
    leftSpeed = 0;
    rightSpeed = 0;
    delay(5000);

    state = WAIT_LIGHT;
  }
  else if (state == GO_AHEAD) {
    leftSpeed = 1;
    rightSpeed = 1;
    
    float error = 0;
    float Kp = 0.7;

    // front obstacle
    if (frontUSSensor <= FRONT_US_LIMIT) {

      if (leftUSSensor > rightUSSensor) {
        leftSpeed = -1;
        rightSpeed = 1;
      }
      else {
        leftSpeed = 1;
        rightSpeed = -1;
      }
      state = CHECK_TURN;

    } else {

      if (leftUSSensor < LEFT_US_LIMIT){
        error = (leftUSSensor - LEFT_US_LIMIT) / LEFT_US_LIMIT;
        Serial.print("Left Error: ");
        Serial.println(error);
        rightSpeed =  rightSpeed + error * Kp;
      }

      if (rightUSSensor < RIGHT_US_LIMIT){
        error = (rightUSSensor - RIGHT_US_LIMIT) / RIGHT_US_LIMIT;
        Serial.print("Right Error: ");
        Serial.println(error);
        leftSpeed = leftSpeed + error * Kp;
      }
    }
  }
  else if (state == CHECK_TURN) {
    // Maybe add checkLight here
    bool turninRight = leftSpeed > rightSpeed;
    if (frontUSSensor > FRONT_US_LIMIT && ((turninRight && rightUSSensor < (RIGHT_US_LIMIT + 10)) || (!turninRight && leftUSSensor < (LEFT_US_LIMIT + 10)))) {
      state = GO_AHEAD;
    }
  }

  setSpeed();
  // delay(0.05);
}

void useL298NDriver(float pwm_left, float pwm_right) {
  digitalWrite(pwm_left_pin, HIGH);
  digitalWrite(pwm_right_pin, HIGH);
  // Motor Left Direction
  if (leftSpeed < 0) {
    analogWrite(in_a_pin, -pwm_left);
    digitalWrite(in_b_pin, LOW); 
  }
  else {
    analogWrite(in_b_pin, pwm_left);
    digitalWrite(in_a_pin, LOW); 
  }
  // Motor Right Direction
  if (rightSpeed < 0) {
    analogWrite(in_c_pin, -pwm_right);
    digitalWrite(in_d_pin, LOW); 
  }
  else {
    analogWrite(in_d_pin, pwm_right);
    digitalWrite(in_c_pin, LOW); 
  }

}

void setSpeed() {
  Serial.print("Setting Speed");
  Serial.print("\tLeft: ");
  Serial.print(leftSpeed);
  Serial.print("\tRight: ");
  Serial.print(rightSpeed);
  Serial.println("");

  useL298NDriver((leftSpeed * 255), (rightSpeed * 255));
}

float readLSSensor(int pin) {
  return analogRead(pin);
}

float readUSSensor(int trigger, int echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(15);
  digitalWrite(trigger, LOW);

  // Get pulse duration in microseconds
  float duration = pulseIn(echo, HIGH);

  // Get distance 
  float distance = SOUND_SPEED  * duration / 2;

  return distance;
}

bool checkLight() {
  if (hasFrontLight()) {
    return true;
  }else if (hasLeftLight()) {
    leftSpeed = -0.7;
    rightSpeed = 0.7;
  }
  else {
    leftSpeed = 0.7;
    rightSpeed = -0.7;
  }
  return false;
}