// Sound Speed
#define SOUND_SPEED 0.0343 // cm/us

// Ultrassonic Sensors Pins
#define ultrasonFrontTrgPin 30  
#define ultrasonFrontEchoPin 31  

#define ultrasonLeftTrgPin 3 //13  
#define ultrasonLeftEchoPin 4 //12  

#define ultrasonRightTrgPin 13 //3  
#define ultrasonRightEchoPin 12 //4

// L298N Driver Pins
#define pwm_right_pin 38
#define pwm_left_pin 48

#define in_a_pin 46
#define in_b_pin 44
#define in_c_pin 42
#define in_d_pin 40

// Ultrassonic Limits
#define FRONT_US_LIMIT 15
#define LEFT_US_LIMIT 15
#define RIGHT_US_LIMIT 15

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

float leftSpeed = 0.0;
float rightSpeed = 0.0;

// Pre-call functions
float readUSSensor(int trigger, int echo);
void setSpeed();

void setup() {
  // Init Serial Comm.
  Serial.begin(115200);

  // Setup L298N Driver Pins
  pinMode(pwm_right_pin, OUTPUT);
  pinMode(in_a_pin, OUTPUT);
  pinMode(in_b_pin, OUTPUT);
  pinMode(in_c_pin, OUTPUT);
  pinMode(in_d_pin, OUTPUT);
  pinMode(pwm_left_pin, OUTPUT);

  // Setup Ultrassonic Sensors Pins
  pinMode(ultrasonFrontTrgPin, OUTPUT);
  pinMode(ultrasonLeftTrgPin, OUTPUT);
  pinMode(ultrasonRightTrgPin, OUTPUT);

  pinMode(ultrasonFrontEchoPin, INPUT);
  pinMode(ultrasonLeftEchoPin, INPUT);
  pinMode(ultrasonRightEchoPin, INPUT);
}

void loop() {
  // Upload sensors value
  frontUSSensor = readUSSensor(ultrasonFrontTrgPin, ultrasonFrontEchoPin);
  leftUSSensor = readUSSensor(ultrasonLeftTrgPin, ultrasonLeftEchoPin);
  rightUSSensor = readUSSensor(ultrasonRightTrgPin, ultrasonRightEchoPin);

  Serial.print("Front: ");
  Serial.print(frontUSSensor);
  Serial.print("\tLeft: ");
  Serial.print(leftUSSensor);
  Serial.print("\tRight: ");
  Serial.println(rightUSSensor);

  // State Machine

  if (state == STOP) {
    leftSpeed = 0;
    rightSpeed = 0;
    delay(5000);

    state = GO_AHEAD;
  }
  else if (state == GO_AHEAD) {
    leftSpeed = 1;
    rightSpeed = 0.6;
    
    float error = 0;
    float Kp = 1;

    // Front Obstacle Detected
    if (frontUSSensor <= FRONT_US_LIMIT) {

      // If left has more space, turn left
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

      // Proportional Control to Avoid lateral walls

      if (leftUSSensor < LEFT_US_LIMIT){
        error = (leftUSSensor - LEFT_US_LIMIT) / LEFT_US_LIMIT;
        rightSpeed =  rightSpeed + error * Kp;
      }

      if (rightUSSensor < RIGHT_US_LIMIT){
        error = (rightUSSensor - RIGHT_US_LIMIT) / RIGHT_US_LIMIT;
        leftSpeed = leftSpeed + error * Kp * 1.3;
      }
    }
  }
  else if (state == CHECK_TURN) {
    bool turninRight = leftSpeed > rightSpeed;
    if (frontUSSensor > (FRONT_US_LIMIT * 1.5)) {
      state = GO_AHEAD;
    }
  }

  setSpeed();
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

  // Set Speed (0 - 255) 
  useL298NDriver((leftSpeed * 255), (rightSpeed * 255));
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
