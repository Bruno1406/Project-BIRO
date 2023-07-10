#include <Servo.h>
#include <SoftwareSerial.h>
#include <SoftPWM.h>

// C++ code
//

/*****************************************
 * Classes Prototypes and Types Definitions
 *****************************************/

class dc_motors_t {
  public:
    dc_motors_t(uint8_t pwm_pin, uint8_t direction_pin)
        : pwm_pin(pwm_pin), direction_pin(direction_pin) {
          SoftPWMSet(this->pwm_pin, 0);
    }

    /**
     * @brief Sets motors speed
     * 
     * @param speed The desired speed for the motor
     */
    void motors_set_speed(int8_t speed);

  private:
    uint8_t pwm_pin;
    uint8_t direction_pin;

};

class distance_sensors_t {
  public:
    distance_sensors_t(uint8_t echo_pin, uint8_t trigger_pin)
        : echo_pin(echo_pin), trigger_pin(trigger_pin) {}

    /**
     * @brief Gets distance reading from hc-sr04 ultrassonic distance sensors
     * 
     * @return The distance reading obtained from the sensor
     */
    uint16_t hcsr04_get_distance();
  
    /**
     * @brief Gets if at least one distance sensor is seeing the wall
     * 
     * @return Whether or not one of the sensors is not seeing the wall
     */
    bool hcsr04_is_not_seeing_wall();

  private:
    uint16_t num_of_readings = 0;
    uint8_t echo_pin;
    uint8_t trigger_pin;
};

class line_sensors_t {
  public:
    line_sensors_t(uint8_t reading_adc_pin)
        : reading_adc_pin(reading_adc_pin) {}

    /**
     * @brief Gets reading from TCRT5000 IR sensor
     * 
     * @return The raw ADC reading from TCRT5000 IR sensor
     */
    uint16_t TCR5000_get_raw_reading();

    /**
     * @brief Gets if TCRT5000 IR sensor is on line
     * 
     * @return Whether or not the sensor is on line
     */
    bool TCR5000_is_on_line();

  private:
      uint8_t reading_adc_pin;
};

struct pid_struct_t {
  float last_error = 0;
  float integral = 0;
  float derivative = 0;
};
typedef enum {
  WAIT_FOR_START,
  FOLLOW_LINE,
  FIRST_TURN,
  GRAB_OBJECTS,
  FORK,
  WAIT_FOR_RC
} autonomous_state_t;

typedef enum {
  IDLE,
  AUTONOMOUS,
  RC,
} main_state_t;

typedef enum {
  LEFT = 1,
  RIGHT = -1
} direction_t;

/*****************************************
 * Constants
 *****************************************/

// DC MOTORS
#define MOTOR_LEFT_PWM_PIN 7
#define MOTOR_LEFT_DIRECTION_PIN 8
 
#define MOTOR_RIGHT_PWM_PIN 9
#define MOTOR_RIGHT_DIRECTION_PIN 10

#define MAX_SPEED 100
#define MIN_SPEED -100

#define RC_FULL_SPEED 100
#define RC_SLOW_SPEED 70
#define RC_TURNING_SPEED 0
#define FOLLOW_LINE_SPEED 50
#define TURN_SPEED 100
#define LEFT_FACTOR 0.7
#define RIGHT_FACTOR 1

// SERVO MOTOR
#define SERVO_PIN 6

#define BARRIER_UP_ANGLE 90
#define BARRIER_DOWN_ANGLE 20

// DISTANCE SENSORS
#define DS_LEFT_ECHO_PIN 2
#define DS_LEFT_TRIGGER_PIN 3

#define DS_RIGHT_ECHO_PIN 4
#define DS_RIGHT_TRIGGER_PIN 5

#define DISTANCE_SENSORS_THRESHOLD 30

// LINE SENSORS
#define LS_FAR_LEFT_READ_PIN A5
#define LS_CENTER_LEFT_READ_PIN A4
#define LS_CENTER_RIGHT_READ_PIN A3
#define LS_FAR_RIGHT_READ_PIN A2

#define NUM_OF_LS 4
#define LINE_SENSORS_THRESHOLD 200

// BLUETOOTH
#define TX 12
#define RX 11

// PID CONSTANTS
#define KP 40 
#define KD 0
#define KI 0

// TIME CONSTANTS MS
#define AUTONOMOUS_MODE_TIME 60000
#define GRAB_OBJECT_ROUTINE_START 10000
#define GRAB_OBJECT_ROUTINE_END 10200
#define FORK_TIME 2000
#define FIRST_TURN_TIME 700

// TESTS
#define MAIN 0
#define TEST_LS 1
#define TEST_DS 2
#define TEST_MOTORS 3
#define TEST_SERVO 4
#define TEST_BLUETOOTH 5
#define TEST_FOLLOW_LINE 6

#define TEST MAIN

/*****************************************
 * Variables
 *****************************************/

static dc_motors_t motor_left(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIRECTION_PIN);
static dc_motors_t motor_right(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIRECTION_PIN);

static distance_sensors_t ds_left(DS_LEFT_ECHO_PIN, DS_LEFT_TRIGGER_PIN);
static distance_sensors_t ds_right(DS_RIGHT_ECHO_PIN, DS_RIGHT_TRIGGER_PIN);

static line_sensors_t ls_array[NUM_OF_LS] = {
  line_sensors_t(LS_FAR_LEFT_READ_PIN),
  line_sensors_t(LS_CENTER_LEFT_READ_PIN),
  line_sensors_t(LS_CENTER_RIGHT_READ_PIN),
  line_sensors_t(LS_FAR_RIGHT_READ_PIN)
};

static int8_t ls_weight[NUM_OF_LS] = {-2,-1,1,2};

static Servo barrier_servo;
static SoftwareSerial bt(RX,TX);

static pid_struct_t pid;

static char command;
static uint8_t angle = BARRIER_UP_ANGLE;
static bool force_rc_mode = false;
static bool is_full_speed = false;
static bool objects_grabbed = false;
static bool is_before_first_turn = true;
static uint32_t first_turn_start_time;
static direction_t fork_direction;
static uint32_t start_time;
static main_state_t state = IDLE;
static autonomous_state_t auto_state = WAIT_FOR_START;

/*****************************************
 * Functions Prototypes
 *****************************************/

/**
 * @brief Sets servo angle
 * 
 * @param servo Struct containing servo information
 * @param angle The desired speed for the servo motor
 */
void servo_set_angle(Servo servo, uint8_t angle);

/**
 * @brief Gets command from bluetooth app
 * 
 * @return The command from the app
 */
char bluetooth_get_command();

/**
 * @brief PID algorithm for robot to follow the line.
 * 
 * @param error Contains information about how off the robot is from the line
 * 
 * @return How much the robot should be turning to get back on track
 */
int16_t pid_algorithm(pid_struct_t&  pid, float error);

/**
 * @brief Autonomous mode Finite State Machine
 *
 */
void autonomous_mode_FSM();

/**
 * @brief Main Finite State Machine
 *
 */
void main_FSM();

/*****************************************
 * Main Functions Definitions
 *****************************************/

void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  SoftPWMBegin();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTOR_LEFT_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIRECTION_PIN, OUTPUT);
  pinMode(DS_LEFT_TRIGGER_PIN, OUTPUT);
  pinMode(DS_RIGHT_TRIGGER_PIN, OUTPUT);
  pinMode(DS_LEFT_ECHO_PIN, INPUT);
  pinMode(DS_RIGHT_ECHO_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
  barrier_servo.attach(SERVO_PIN);
  servo_set_angle(barrier_servo, BARRIER_UP_ANGLE);
}

void loop() {
#if TEST == MAIN
  Serial.println(auto_state);
  main_FSM();   

#elif TEST == TEST_LS
  for (uint8_t i = 0; i < NUM_OF_LS; i++) {
    Serial.print("Line Sensor ");
    Serial.print(i);
    Serial.println(":");
    Serial.print(ls_array[i].TCR5000_get_raw_reading());
    Serial.print(" : ");
    Serial.println(ls_array[i].TCR5000_is_on_line());
  }
  

#elif TEST == TEST_DS
  Serial.print("Distance Sensor Left: ");
  Serial.println(ds_left.hcsr04_get_distance());
  Serial.println(ds_left.hcsr04_is_not_seeing_wall());
  Serial.print("Distance Sensor Right: ");
  Serial.println(ds_right.hcsr04_get_distance());
  Serial.println(ds_right.hcsr04_is_not_seeing_wall());
  delay(1000);

#elif TEST == TEST_MOTORS
  motor_left.motors_set_speed(MAX_SPEED);
  motor_right.motors_set_speed(MAX_SPEED);
  delay(3000);
  motor_left.motors_set_speed(MIN_SPEED);
  motor_right.motors_set_speed(MIN_SPEED);
  delay(3000);
  motor_left.motors_set_speed(MAX_SPEED);
  motor_right.motors_set_speed(MIN_SPEED);
  delay(3000);
  motor_left.motors_set_speed(MIN_SPEED);
  motor_right.motors_set_speed(MAX_SPEED);
  delay(3000);

#elif TEST == TEST_SERVO
  servo_set_angle(barrier_servo, BARRIER_DOWN_ANGLE);
  delay(3000);
  servo_set_angle(barrier_servo, BARRIER_UP_ANGLE);
  delay(3000);

#elif TEST == TEST_BLUETOOTH
  char command = bluetooth_get_command();
  if (command != '0') {
    Serial.println(command);
  }
#elif TEST == TEST_FOLLOW_LINE 
  auto_state = FOLLOW_LINE;
  main_FSM();
#endif 
}

/*****************************************
 * Class Methods Definitions
 *****************************************/

void dc_motors_t::motors_set_speed(int8_t speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  uint8_t duty_cycle = abs(speed);
  speed >= 0 ? digitalWrite(this->direction_pin, HIGH) : digitalWrite(this->direction_pin, LOW);
  SoftPWMSetPercent(this->pwm_pin,duty_cycle);
}

uint16_t distance_sensors_t::hcsr04_get_distance() {
  digitalWrite(this->trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->trigger_pin, LOW);
  // Converts time in Microsseconds to centimeters
  return (pulseIn(this->echo_pin, HIGH)/58);        
}

bool distance_sensors_t::hcsr04_is_not_seeing_wall() {
  num_of_readings = (hcsr04_get_distance() > DISTANCE_SENSORS_THRESHOLD) ? (num_of_readings + 1) : 0;
  return num_of_readings > 5; 
}

bool line_sensors_t::TCR5000_is_on_line() {
  return (analogRead(this->reading_adc_pin) > LINE_SENSORS_THRESHOLD);
}

uint16_t line_sensors_t::TCR5000_get_raw_reading() {
  return analogRead(this->reading_adc_pin);
}

/*****************************************
 * Functions Definitions
 *****************************************/

void servo_set_angle(Servo servo, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  servo.write(angle);
}

char bluetooth_get_command() {
  static char command;
  if (bt.available()) {
    command = bt.read();
  }
  if (Serial.available()) {
      bt.write(Serial.read());
  }
  return command;
}

int16_t pid_algorithm(pid_struct_t& pid, float error) {
  pid.derivative = error - pid.last_error;
  pid.integral += error;
  pid.last_error = error;
  int16_t response = (int16_t) (KP*error + KD*pid.derivative + KI*pid.integral);
  return response;
}

void autonomous_mode_FSM() {
  uint32_t time = millis() - start_time;
  switch (auto_state) {
    case WAIT_FOR_START: {
      if (command == 'C') {
        auto_state = FOLLOW_LINE;
        start_time = millis();
      } else {
        auto_state = WAIT_FOR_START;
      }
      break;
    }
    case FOLLOW_LINE: {
      if (time > AUTONOMOUS_MODE_TIME) {
        force_rc_mode = true;
      }
      uint8_t num_of_oN_line_ls = 0;
      uint8_t error = 0;
      for (uint8_t i = 0; i < NUM_OF_LS; i++) {
        if (ls_array[i].TCR5000_is_on_line()) {
          error += ls_weight[i];
          num_of_oN_line_ls++;
        }
      }
      if (error < 0) {
        motor_left.motors_set_speed(LEFT_FACTOR*FOLLOW_LINE_SPEED);
        motor_right.motors_set_speed(-RIGHT_FACTOR*FOLLOW_LINE_SPEED);
      } else if (error > 0) {
        motor_left.motors_set_speed(-LEFT_FACTOR*FOLLOW_LINE_SPEED);
        motor_right.motors_set_speed(RIGHT_FACTOR*FOLLOW_LINE_SPEED);
      } else {
        motor_left.motors_set_speed(LEFT_FACTOR*FOLLOW_LINE_SPEED);
        motor_right.motors_set_speed(RIGHT_FACTOR*FOLLOW_LINE_SPEED);
      }
      if (is_before_first_turn) {
        if(ds_left.hcsr04_is_not_seeing_wall()) {
          fork_direction = LEFT;
          auto_state = FIRST_TURN;
        } else if (ds_right.hcsr04_is_not_seeing_wall()) {
          fork_direction = RIGHT;
          auto_state = FIRST_TURN;
        }
      } else {
        auto_state = FOLLOW_LINE;
      }
      break;
    }
    case FIRST_TURN: {
      if (is_before_first_turn) {
        first_turn_start_time = millis();
        is_before_first_turn = false;
      }
      if (millis() - first_turn_start_time < FIRST_TURN_TIME) {
        motor_left.motors_set_speed(-LEFT_FACTOR * fork_direction * TURN_SPEED);
        motor_right.motors_set_speed(RIGHT_FACTOR * fork_direction * TURN_SPEED);
      } else {
        auto_state = FOLLOW_LINE;
      }
    }
  }
}

void main_FSM() {
  command = bluetooth_get_command();
  switch(state) {
    case IDLE: {
     motor_left.motors_set_speed(0);
      motor_right.motors_set_speed(0);
      servo_set_angle(barrier_servo, BARRIER_UP_ANGLE);
      if (command == '1') {
        state = AUTONOMOUS;
        auto_state = WAIT_FOR_START;
      } else if (command == '2') {
        state = RC;
      } else {
        state = IDLE;
      }
      break;
    }
    case RC: {
      if (command == 'B') {
          is_full_speed = false;
      }
      if (command == 'A') {
        is_full_speed = true;
      }
      uint8_t speed = is_full_speed ? RC_FULL_SPEED : RC_SLOW_SPEED; 
      if (command == 'D') {
        angle = angle != BARRIER_UP_ANGLE ? BARRIER_UP_ANGLE : BARRIER_DOWN_ANGLE;
        servo_set_angle(barrier_servo, angle);
      }
      if (command == 'a') {
        motor_left.motors_set_speed(LEFT_FACTOR * speed);
        motor_right.motors_set_speed(RIGHT_FACTOR * speed);
      }
      if (command == 'k') {
       motor_left.motors_set_speed(LEFT_FACTOR * speed);
       motor_right.motors_set_speed(0);
      }
      if (command == 'l') {
       motor_left.motors_set_speed(-LEFT_FACTOR * speed);
       motor_right.motors_set_speed(0);
      }
      if (command == 'm') {
       motor_left.motors_set_speed(0);
       motor_right.motors_set_speed(-RIGHT_FACTOR * speed);
      }
      if (command == 'n') {
       motor_left.motors_set_speed(0);
       motor_right.motors_set_speed(RIGHT_FACTOR * speed);
      }  
      if (command == 'c') {
       motor_left.motors_set_speed(-LEFT_FACTOR * speed);
       motor_right.motors_set_speed(-RIGHT_FACTOR * speed);
      } 
      if (command == 'd') {
       motor_left.motors_set_speed(-LEFT_FACTOR * speed);
       motor_right.motors_set_speed(RIGHT_FACTOR * speed);
      } 
      if (command == 'b') {
       motor_left.motors_set_speed(LEFT_FACTOR * speed);
       motor_right.motors_set_speed(-RIGHT_FACTOR * speed);
      } 
      if (command == '0') {
       motor_left.motors_set_speed(0);
       motor_right.motors_set_speed(0);
      }
      if (command == '3') {
        state = IDLE;
      } else if (command == '1') {
        state = AUTONOMOUS;
        auto_state = WAIT_FOR_START;
      } else {
        state = RC;
      }
      break;
    }
    case AUTONOMOUS: {
      autonomous_mode_FSM();
      if (command == '2' || force_rc_mode) {
        state = RC;
      } else if (command == '3') {
        state = IDLE;
      } else {
        state = AUTONOMOUS;
      }
      break;
    }
  }
}
