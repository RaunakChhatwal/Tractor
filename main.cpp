#include <IRremote.h>
#include <Servo.h>

int ultrasonic_delay = 10;  // in ms
int IR_delay(100), start_delay(250);
int frame_delay = 100 - ultrasonic_delay;
int led = 4;
Servo my_servo;
int servo_pin(11), servo_pos;
float safe_distance = 7;  // in inches
int IR_pin(10), five_volts(2);
IRrecv IR(IR_pin);
decode_results cmd;

struct Motor {
  int pin_1, pin_2, en;
  Motor(int _pin_1, int _pin_2, int _en) { pin_1 = _pin_1; pin_2 = _pin_2; en = _en; }
  
  void power(const int& voltage, const bool& forward = true) {
    if (forward) {
      digitalWrite(pin_1, LOW);
      digitalWrite(pin_2, HIGH);
    } else {
      digitalWrite(pin_1, HIGH);
      digitalWrite(pin_2, LOW);
    }

    analogWrite(en, voltage);
  }

  void set_pin_mode() {
    pinMode(pin_1, OUTPUT);
    pinMode(pin_2, OUTPUT);
    pinMode(en, OUTPUT);
  }
};

Motor left_motor{6, 7, 3}, right_motor{8, 9, 5};

bool to_stop();
void change_direction(const bool& _direction, const int& degrees_per_second);
void move_robot(const int& _speed, const bool& forward = true);
void accelerate_robot(const int& _speed, const bool& forward = true);

int trig_pin(12), echo_pin(13);

void setup() {
  pinMode(five_volts, OUTPUT);
  digitalWrite(five_volts, HIGH);
  IR.enableIRIn();
  
  left_motor.set_pin_mode();
  right_motor.set_pin_mode();
  

  Serial.begin(9600);

  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  my_servo.attach(servo_pin);

  pinMode(led, OUTPUT);
}

int robot_speed = 0;
int target_speed = 255;
bool robot_forward = true;
int robot_angle = 0;
bool robot_direction;
bool just_stopped;
bool start_mode = true;
void loop() {
  
  if (just_stopped = to_stop()) {
    robot_speed = 0;
    start_mode = true;
  }
  
  if (IR.decode(&cmd)) {
    
    Serial.println(cmd.value, HEX);

    if (start_mode) {
      switch (cmd.value) {

        // forward
        case 0xFF18E7: case 0x3D9AE3F7: if (!just_stopped) {
          robot_forward = true;
          robot_speed = target_speed;
          accelerate_robot(robot_speed, robot_forward);
          start_mode = false;
        }
        break;

        // right
        case 0xFF5AA5: case 0x449E79F: {
          robot_direction = true;
          robot_angle = 15;
          break;
        }

        // backward
        case 0xFF4AB5: case 0x1BC0157B: {
          robot_forward = false;
          robot_speed = target_speed;
          accelerate_robot(robot_speed, robot_forward);
          start_mode = false;
          break;
        }

        // left
        case 0xFF10EF: case 0x8C22657B: {
          robot_direction = false;
          robot_angle = 15;
          break;
        }

        // OK (Center button)
        case 0xFF38C7: case 0x488F3CBB: {
          robot_angle = 0;
          break;
        }
        
      }
    } else {
      robot_speed = 0;
      robot_angle = 0;
      start_mode = true;
    }
    
    IR.resume();
  }
  
  move_robot(robot_speed, robot_forward);
  change_direction(robot_direction, robot_angle);

  if (robot_forward) digitalWrite(led, LOW); else digitalWrite(led, HIGH);
  
  delay(frame_delay);
}

// define get_direction
int get_direction() { return 0; }

// define to_stop, make sure to update this for different speeds
bool to_stop() {
  if (!robot_forward) return false;
  
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(ultrasonic_delay);
  digitalWrite(trig_pin, LOW);

  // measure duration of pulse from ECHO pin
  float distance_1 = 0.0067 * pulseIn(echo_pin, HIGH);

  if (distance_1 < 2 * safe_distance) {
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(ultrasonic_delay);
    digitalWrite(trig_pin, LOW);
    
    // measure duration of pulse from ECHO pin
    float distance_2 = 0.0067 * pulseIn(echo_pin, HIGH);
  
    return (distance_2 - distance_1) * ultrasonic_delay > (safe_distance / 2.5) * 1000 || distance_1 < safe_distance;
  }

  return false;
}

// define get_speed
float get_speed() {
  return 0;
}

// define change_direction
void change_direction(const bool& _direction, const int& degrees_per_second) {
  if (_direction) servo_pos = 80 + 7 * degrees_per_second / 3;
  else servo_pos = 80 - 7 * degrees_per_second / 3;

  my_servo.write(servo_pos);
}

// define move_robot
void move_robot(const int& _speed, const bool& forward) {
  left_motor.power(_speed, forward);
  right_motor.power(_speed, forward);
}

void accelerate_robot(const int& _speed, const bool& forward) {
  move_robot(255, forward);
  delay(start_delay);
}
