#define DECODE_NEC
#include <IRremote.hpp>

// Sound Speed
#define SOUND_SPEED 0.0343 // cm/us
#define PULSE_TIMEOUT 100000 // 100000 us (100ms);

// Ultrassonic Sensors Pins
#define ultrasonFrontTrgPin 4
#define ultrasonFrontEchoPin 5  

#define ultrasonLeftTrgPin 2
#define ultrasonLeftEchoPin 3

#define ultrasonRightTrgPin 6
#define ultrasonRightEchoPin 7

// L298N Driver Pins
#define pwm_right_pin 8
#define pwm_left_pin 13

#define in_a_pin 12
#define in_b_pin 11
#define in_c_pin 10
#define in_d_pin 9

// LDRs Pins
#define ldr_back_right_pin A15
#define ldr_back_left_pin A14
#define ldr_front_right_pin A0
#define ldr_front_left_pin A1

// IR Remote
#define IR_RECEIVE_PIN 45
#define ENABLE_LED_FEEDBACK 47

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

enum operationMode {
  BRAITENGER_FOLLOW = 0,
  BRAITENGER_HIDE = 1,
  REACTIVE = 2,
};

enum IRControlMap {
  ONE     = 0x45,
  TWO     = 0x46,
  THREE   = 0x47,
  FOUR    = 0x44,
  FIVE    = 0x40,
  SIX     = 0x43,
  SEVEN   = 0x07,
  EIGHT   = 0x15,
  NINE    = 0x09,
  ASTERISK= 0x16,
  ZERO    = 0x19,
  HASHTAG = 0x0D,
  UP      = 0x18,
  LEFT    = 0x08,
  RIGHT   = 0x5A,
  DOWN    = 0x52,
  OK      = 0X1C
};

bool stopAll = true;
bool enableLog = false;

operationState state = STOP;
operationMode mode = operationMode::REACTIVE;

// Ultrassonic Sensors Handlers
float frontUSSensor = 0;
float leftUSSensor = 0;
float rightUSSensor = 0;

float leftSpeed = 0.0;
float rightSpeed = 0.0;

// LDRs Pins
uint16_t ldr_back_left = 0;
uint16_t ldr_back_right = 0;
uint16_t ldr_front_left = 0;
uint16_t ldr_front_right = 0;

// Pre-call functions
float readUSSensor(int trigger, int echo);
void setSpeed();
uint16_t readLDRSensor(int pin, uint16_t * value);

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

  // Setup LDR Pins
  pinMode(ldr_front_left_pin, INPUT);
  pinMode(ldr_front_right_pin, INPUT);
  pinMode(ldr_back_left_pin, INPUT);
  pinMode(ldr_back_right_pin, INPUT);

  // IR Remote setup
  IrReceiver.begin(IR_RECEIVE_PIN, true, ENABLE_LED_FEEDBACK); // Start the receiver
  
}

void loop() {
  // Upload sensors value
  delay(100);

  if (IrReceiver.decode()) {
    Serial.println("******************** DECODING IR ************************");

    if (IrReceiver.decodedIRData.command == IRControlMap::ONE) 
    {
      Serial.println("One pressed, setting reactive");
      mode = operationMode::REACTIVE;
    }
    else if (IrReceiver.decodedIRData.command == IRControlMap::TWO) 
    {
      Serial.println("Two pressed setting baiterberg follow");
      mode = operationMode::BRAITENGER_FOLLOW;
    }
    else if (IrReceiver.decodedIRData.command == IRControlMap::THREE) 
    {
      Serial.println("Three pressed setting baiterberg hide");
      mode = operationMode::BRAITENGER_HIDE;
    }
    else if (IrReceiver.decodedIRData.command == IRControlMap::OK) 
    {
      Serial.println("OK pressed, toggling baiterberg follow");
      stopAll = !stopAll;
    }
    else if (IrReceiver.decodedIRData.command == IRControlMap::HASHTAG)
    {
      Serial.println("HASTAG pressed, toggling LOG mode");
      enableLog = !enableLog;
    }
    else 
    {
      IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    }

    Serial.print("Protocol: ");
    Serial.println(IrReceiver.getProtocolString());
    
    IrReceiver.resume(); // Enable receiving of the next value
  }

  if (mode == BRAITENGER_FOLLOW || mode == BRAITENGER_HIDE)
  {
    bool doFollow = mode == BRAITENGER_FOLLOW;

    readLDRSensor(ldr_front_right_pin, &ldr_front_right);
    readLDRSensor(ldr_front_left_pin, &ldr_front_left);
    readLDRSensor(ldr_back_right_pin, &ldr_back_right);
    readLDRSensor(ldr_back_left_pin, &ldr_back_left);

    uint16_t front_light = ldr_front_right + ldr_front_left;
    uint16_t back_light = ldr_back_right + ldr_back_left;
    uint16_t left_light = ldr_back_left + ldr_front_left;
    uint16_t right_light = ldr_back_right + ldr_front_right;

    Serial.print("[LDR]: ");
    Serial.print(front_light);
    Serial.print("\t");
    Serial.print(back_light);
    Serial.print("\t");
    Serial.print(left_light);
    Serial.print("\t");
    Serial.println(right_light);

    if (front_light < 450 && 
        front_light < back_light && 
        front_light < left_light &&
        front_light < right_light) 
    {
      leftSpeed = 1.0;
      rightSpeed = 1.0;
    }
    else if (back_light < 450 && 
            back_light < front_light && 
            back_light < left_light &&
            back_light < right_light)
    {
      leftSpeed = -1.0;
      rightSpeed = -1.0;
    }
    else if (left_light < 450 && 
            left_light < front_light && 
            left_light < back_light &&
            left_light < right_light)
    {
      leftSpeed = -1.0;
      rightSpeed = 1.0;
    }
    else if (right_light < 450 && 
            right_light < front_light && 
            right_light < left_light &&
            right_light < back_light)
    {
      leftSpeed = 1.0;
      rightSpeed = -1.0;
    }
    else 
    {
      leftSpeed = 0;
      rightSpeed = 0;
    }

    

    // leftSpeed -= ldr_front_right / 1024.0;
    // rightSpeed -= ldr_front_left / 1024.0;

    // leftSpeed *= (front_light > back_light && doFollow) ? 1 : -1;
    // rightSpeed *= (front_light > back_light && doFollow) ? 1 : -1;

  }
  else
  {
    frontUSSensor = readUSSensor(ultrasonFrontTrgPin, ultrasonFrontEchoPin);
    leftUSSensor = readUSSensor(ultrasonLeftTrgPin, ultrasonLeftEchoPin);
    rightUSSensor = readUSSensor(ultrasonRightTrgPin, ultrasonRightEchoPin);
    
    Serial.print("[DISTANCE] Front: ");
    Serial.print(frontUSSensor);
    Serial.print("[DISTANCE] \tLeft: ");
    Serial.print(leftUSSensor);
    Serial.print("[DISTANCE] \tRight: ");
    Serial.println(rightUSSensor);

    // State Machine
    if (state == STOP) 
    {
      leftSpeed = 0;
      rightSpeed = 0;
      delay(5000);

      state = GO_AHEAD;
    }
    else if (state == GO_AHEAD) 
    {
      leftSpeed = 1;
      rightSpeed = 1;
      
      float error = 0;
      float Kp = 1;

      // Front Obstacle Detected
      if (frontUSSensor <= FRONT_US_LIMIT) 
      {

        // If left has more space, turn left
        if (leftUSSensor > rightUSSensor) 
        {
          leftSpeed = -1;
          rightSpeed = 1;
        }
        else 
        {
          leftSpeed = 1;
          rightSpeed = -1;
        }
        state = CHECK_TURN;

      } else 
      {
        // Proportional Control to Avoid lateral walls

        if (leftUSSensor < LEFT_US_LIMIT)
        {
          error = (leftUSSensor - LEFT_US_LIMIT) / LEFT_US_LIMIT;
          rightSpeed =  rightSpeed + error * Kp;
        }

        if (rightUSSensor < RIGHT_US_LIMIT)
        {
          error = (rightUSSensor - RIGHT_US_LIMIT) / RIGHT_US_LIMIT;
          leftSpeed = leftSpeed + error * Kp * 1.3;
        }
      }
    }
    else if (state == CHECK_TURN)
    {
      bool turninRight = leftSpeed > rightSpeed;
      if (frontUSSensor > (FRONT_US_LIMIT * 1.5))
      {
        state = GO_AHEAD;
      }
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
  if (stopAll)
  {
    leftSpeed = 0.0;
    rightSpeed = 0.0;
  }
  else 
  {
    Serial.print("Setting Speed");
    Serial.print("\tLeft: ");
    Serial.print(leftSpeed);
    Serial.print("\tRight: ");
    Serial.print(rightSpeed);
    Serial.println("");
  }

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
  float duration = pulseIn(echo, HIGH, PULSE_TIMEOUT);

  // Get distance 
  float distance = SOUND_SPEED  * duration / 2;

  return distance;
}

uint16_t readLDRSensor(int pin, uint16_t * value) {
  *value = analogRead(pin);
}
