#include <Servo.h>
#include <SoftwareSerial.h>
#include <SoftPWM.h>

// C++ code
//

/*****************************************
 * Types Definitions
 *****************************************/

struct dc_motors_t {
  uint8_t pwm_pin;
  uint8_t direction_pin;
  dc_motors_t(uint8_t pwm_pin, uint8_t direction_pin)
      : pwm_pin(pwm_pin), direction_pin(direction_pin) {}
};

struct distance_sensors_t {
  uint8_t echo_pin;
  uint8_t trigger_pin;
  distance_sensors_t(uint8_t echo_pin, uint8_t trigger_pin)
      : echo_pin(echo_pin), trigger_pin(trigger_pin) {}
};

struct line_sensors_t {
  uint8_t reading_adc_pin;
  line_sensors_t(uint8_t reading_adc_pin)
      : reading_adc_pin(reading_adc_pin) {}
};

typedef enum {
  WAIT_FOR_START,
  FOLLOW_LINE,
  FIRST_TURN,
  GRAB_OBJECTS,
  CROSS_H
} autonomous_state_t;

typedef enum {
  IDLE,
  AUTONOMOUS,
  RC,
} main_state_t;

/*****************************************
 * Constants
 *****************************************/

// DC MOTORS
#define MOTOR_LEFT_PWM_PIN 9
#define MOTOR_LEFT_DIRECTION_PIN 10
 
#define MOTOR_RIGHT_PWM_PIN 7
#define MOTOR_RIGHT_DIRECTION_PIN 8

#define MAX_SPEED 100
#define MIN_SPEED -100

#define RC_FULL_SPEED 80
#define RC_TURNING_SPEED 80
#define FOLLOW_LINE_SPEED 30
#define CROSS_H_TURNING_SPEED 70
#define CROSS_H_STRAIGHT_SPEED 50
#define FIRST_TURN_TURNING_SPEED 70
#define FIRST_TURN_STRAIGHT_SPEED 50

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
#define KP 0
#define KD 0
#define KI 0

// TIME CONSTANTS MS
#define AUTONOMOUS_MODE_TIME 60000
#define GRAB_OBJECT_ROUTINE_START 10000
#define GRAB_OBJECT_ROUTINE_END 120000
#define CROSS_H_DECELERATION 100
#define CROSS_H_TURNING CROSS_H_DECELERATION + 300
#define CROSS_H_STRAIGHT_LINE CROSS_H_TURNING + CROSS_H_DECELERATION + 2000
#define FIRST_TURN_DECELERATION 100
#define FIRST_TURN_STRAIGHT_LINE 200
#define FIRST_TURN_TURNING FIRST_TURN_STRAIGHT_LINE + FIRST_TURN_DECELERATION + 1000
#define DEBOUNCE_TIME 100
#define SAFETY_TIME 1500

// TESTS
#define MAIN 0
#define TEST_LS 1
#define TEST_DS 2
#define TEST_MOTORS 3
#define TEST_SERVO 4
#define TEST_BLUETOOTH 5

#define TEST TEST_DS



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

static char command;
static bool force_rc_mode = false;

/*****************************************
 * Functions Prototypes
 *****************************************/

/**
 * @brief Sets motors speed
 * 
 * @param motor Struct containing motor PWM pins
 * @param speed The desired speed for the motor
 */
void motors_set_speed(dc_motors_t motor, int8_t speed);

/**
 * @brief Sets servo angle
 * 
 * @param servo Struct containing servo information
 * @param angle The desired speed for the servo motor
 */
void servo_set_angle(Servo servo, uint8_t angle);

/**
 * @brief Gets distance reading from hc-sr04 ultrassonic distance sensors
 * 
 * @param hcsr04_sensor Struct containing distance sensor information
 * 
 * @return The distance reading obtained from the sensor
 */
uint16_t hcsr04_get_distance(distance_sensors_t hcTCR5000sr04_sensor);

/**
 * @brief Gets if at least one distance sensor is seeing the wall
 * 
 * @return Whether or not one of the sensors is not seeing the wall
 */
bool hcsr04_is_not_seeing_wall();

/**
 * @brief Gets reading from TCRT5000 IR sensor
 * 
 * @param TCRT5000_sensor Struct containing line sensor information
 * 
 * @return Whether or not the sensor is on line
 */
bool TCR5000_is_on_line(line_sensors_t TCR5000_sensor);

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
int16_t pid_algorithm(float error);

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
  SoftPWMSet(motor_left.pwm_pin, 0);
  SoftPWMSet(motor_right.pwm_pin, 0);
  barrier_servo.attach(SERVO_PIN);
  servo_set_angle(barrier_servo, BARRIER_UP_ANGLE);
}

void loop() {
#if TEST == MAIN
  Serial.println("BIRO");
  main_FSM();   

#elif TEST == TEST_LS
  for (uint8_t i = 0; i < NUM_OF_LS; i++) {
    Serial.print("Line Sensor ");
    Serial.print(i);
    Serial.println(":");
    Serial.print(analogRead(ls_array[i].reading_adc_pin));
    Serial.print(" : ");
    Serial.println(TCR5000_is_on_line(ls_array[i]));
  }
  delay(1000);

#elif TEST == TEST_DS
  Serial.print("Distance Sensor Left: ");
  Serial.println(hcsr04_get_distance(ds_left));
  Serial.print("Distance Sensor Right: ");
  Serial.println(hcsr04_get_distance(ds_right));
  Serial.print("Is seeing wall: ");
  Serial.println(hcsr04_is_not_seeing_wall());
  delay(1000);

#elif TEST == TEST_MOTORS
  motors_set_speed(motor_left, MAX_SPEED);
  motors_set_speed(motor_right, MAX_SPEED);
  delay(3000);
  motors_set_speed(motor_left, MIN_SPEED);
  motors_set_speed(motor_right, MIN_SPEED);
  delay(3000);
  motors_set_speed(motor_left, MAX_SPEED);
  motors_set_speed(motor_right, MIN_SPEED);
  delay(3000);
  motors_set_speed(motor_left, MIN_SPEED);
  motors_set_speed(motor_right, MAX_SPEED);
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
#endif 
}

/*****************************************
 * Functions Definitions
 *****************************************/

void motors_set_speed(dc_motors_t motor, int8_t speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  uint8_t duty_cycle = abs(speed);
  speed >= 0 ? digitalWrite(motor.direction_pin, HIGH) : digitalWrite(motor.direction_pin, LOW);
  SoftPWMSetPercent(motor.pwm_pin,duty_cycle);
}

void servo_set_angle(Servo servo, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  servo.write(angle);
}

uint16_t hcsr04_get_distance(distance_sensors_t hcsr04_sensor) {
  digitalWrite(hcsr04_sensor.trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(hcsr04_sensor.trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(hcsr04_sensor.trigger_pin, LOW);
  // Converts time in Microsseconds to centimeters
  return (pulseIn(hcsr04_sensor.echo_pin, HIGH)/58);        
}

bool hcsr04_is_not_seeing_wall() {
  static uint8_t num_of_readings = 0;
  num_of_readings += (hcsr04_get_distance(ds_left) > DISTANCE_SENSORS_THRESHOLD || 
                      hcsr04_get_distance(ds_right) > DISTANCE_SENSORS_THRESHOLD) ? 1 : -num_of_readings;
  return num_of_readings > 5; 
}

bool TCR5000_is_on_line(line_sensors_t TCR5000_sensor) {
  return (analogRead(TCR5000_sensor.reading_adc_pin) > LINE_SENSORS_THRESHOLD);
}

char bluetooth_get_command() {
  static uint32_t debounce_timer = 0; 
  static bool reset_timer = true;
  static char current_command = '0';
  char new_command = '0';
  if (bt.available()) {
    command = bt.read();
  }
  if (Serial.available()) {
      bt.write(Serial.read());
  }
  uint8_t debounce_max_time = new_command == 'C' || new_command == 'A' ? SAFETY_TIME : DEBOUNCE_TIME;
  if (new_command == current_command) {
    reset_timer = true;
  } else {
    if (reset_timer) {
      debounce_timer = millis();
      reset_timer = false;
    }
    if (millis() - debounce_timer > debounce_max_time) {
      current_command = new_command;
    }
  } 
  return current_command;
}

int16_t pid_algorithm(float error) {
  static float last_error = 0;
  static float integral = 0;
  static float derivative = 0;
  derivative = error - last_error;
  integral += error;
  last_error = error;
  int16_t response = (int16_t) (KP*error + KD*derivative + KI*integral);
  return response;
}

void autonomous_mode_FSM() {
  static autonomous_state_t state = WAIT_FOR_START;
  static uint32_t start_time = 0;
  static bool objects_grabbed = false;
  uint32_t time = millis() - start_time;
  if (start_time != 0 && time > AUTONOMOUS_MODE_TIME) {
    force_rc_mode = true;
  }
  switch (state) {
    case WAIT_FOR_START: {
      if (command == 'A') {
        state = FOLLOW_LINE;
        start_time = millis();
      } else {
        state = WAIT_FOR_START;
      }
      break;
    }
    case FOLLOW_LINE: {
      uint8_t num_of_on_line_ls = 0;
      uint8_t error = 0;
      for (uint8_t i = 0; i < NUM_OF_LS; i++) {
        if (TCR5000_is_on_line(ls_array[i])) {
          error += ls_weight[i];
          num_of_on_line_ls++;
        }
      }
      if (num_of_on_line_ls != 0) {
        error /= num_of_on_line_ls;
      }
      int16_t response = pid_algorithm(error);
      motors_set_speed(motor_left, FOLLOW_LINE_SPEED + response);
      motors_set_speed(motor_right, FOLLOW_LINE_SPEED - response);
      if (hcsr04_is_not_seeing_wall() && !objects_grabbed) {
        state = FIRST_TURN;
      } else if (hcsr04_is_not_seeing_wall()) {
        state = CROSS_H;
      } else if (time - start_time > GRAB_OBJECT_ROUTINE_START && time - start_time < GRAB_OBJECT_ROUTINE_END) {
        state = GRAB_OBJECTS;
      } else if (num_of_on_line_ls == 0 && hcsr04_is_not_seeing_wall()) {
        motors_set_speed(motor_left, 0);
        motors_set_speed(motor_right, 0);
        state = WAIT_FOR_START;
      } else {
        state = FOLLOW_LINE;
      }
      break;
    }
    case FIRST_TURN: {
      static uint32_t first_turn_start_time = millis();
      static int8_t direction = hcsr04_get_distance(ds_left) > DISTANCE_SENSORS_THRESHOLD ? -1 : 1;
      if (time - first_turn_start_time < FIRST_TURN_STRAIGHT_LINE) {
        motors_set_speed(motor_left, FIRST_TURN_STRAIGHT_SPEED);
        motors_set_speed(motor_right, FIRST_TURN_STRAIGHT_SPEED);
      } else if (time - first_turn_start_time <= FIRST_TURN_STRAIGHT_LINE + FIRST_TURN_DECELERATION) {
        motors_set_speed(motor_left, 0);
        motors_set_speed(motor_right, 0);
      } else if (time - first_turn_start_time < FIRST_TURN_TURNING) {
        motors_set_speed(motor_left, FIRST_TURN_TURNING_SPEED * direction);
        motors_set_speed(motor_right, -FIRST_TURN_TURNING_SPEED * direction);
      }
      if (time - first_turn_start_time >= FIRST_TURN_TURNING) {
        state = FOLLOW_LINE;
      } else {
        state = FIRST_TURN;
      }
    }
    case CROSS_H: {
      static uint32_t cross_h_start_time = millis();
      static int8_t direction = hcsr04_get_distance(ds_left) > DISTANCE_SENSORS_THRESHOLD ? -1 : 1;
      if (time - cross_h_start_time < CROSS_H_DECELERATION) {
        motors_set_speed(motor_left, 0);
        motors_set_speed(motor_right, 0);
        break;
      } else if (time - cross_h_start_time <= CROSS_H_TURNING) {
        motors_set_speed(motor_left, CROSS_H_TURNING_SPEED * direction);
        motors_set_speed(motor_right, -CROSS_H_TURNING_SPEED * direction);
        break;
      } else if (time - cross_h_start_time < CROSS_H_DECELERATION + CROSS_H_TURNING) {
        motors_set_speed(motor_left, 0);
        motors_set_speed(motor_right, 0);
        break;
      } else {
        motors_set_speed(motor_left, CROSS_H_STRAIGHT_SPEED);
        motors_set_speed(motor_right, CROSS_H_STRAIGHT_SPEED);
      }
      uint8_t num_of_on_line_ls = 0;
      for (uint8_t i = 0; i < NUM_OF_LS; i++) {
        if (TCR5000_is_on_line(ls_array[i])) {
          num_of_on_line_ls++;
        }
      }
      if (time - cross_h_start_time > CROSS_H_STRAIGHT_LINE && num_of_on_line_ls != 0) {
        state = FOLLOW_LINE;
      } else {
        state = CROSS_H;
      }
      break;  
    }
    case GRAB_OBJECTS: {
      servo_set_angle(barrier_servo, BARRIER_DOWN_ANGLE);
      // TODO: GRAB MANEUVER
      if (time - start_time > GRAB_OBJECT_ROUTINE_END) {
        objects_grabbed = true;
        state = FOLLOW_LINE;
      } else {
        state = GRAB_OBJECTS;
      }
      break;
    }
  }
}

void main_FSM() {
  static main_state_t state = IDLE;
  static uint32_t time = 0;
  command = bluetooth_get_command();
  switch(state) {
    case IDLE: {
      motors_set_speed(motor_left, 0);
      motors_set_speed(motor_right, 0);
      servo_set_angle(barrier_servo, BARRIER_UP_ANGLE);
      if (command == 'C' && time == 0) {
        state = AUTONOMOUS;
        time = millis();
      } else if (command == 'C' && time != 0) {
        state = RC;
      } else {
        state = IDLE;
      }
      break;
    }
    case RC: {
      static bool is_full_speed = false;
      static uint8_t angle = BARRIER_UP_ANGLE;
      uint8_t speed_divisor = is_full_speed ? 1 : 2; 
      if (command == 'B') {
          is_full_speed = ~is_full_speed;
      }
      if (command == 'D') {
        angle = angle != BARRIER_UP_ANGLE ? BARRIER_UP_ANGLE : BARRIER_DOWN_ANGLE;
        servo_set_angle(barrier_servo, angle);
      }
      if (command == 'a') {
        motors_set_speed(motor_left, RC_FULL_SPEED/speed_divisor);
        motors_set_speed(motor_right, RC_FULL_SPEED/speed_divisor);
      }
      if (command == 'k') {
        motors_set_speed(motor_left, RC_FULL_SPEED/speed_divisor);
        motors_set_speed(motor_right, RC_TURNING_SPEED/speed_divisor);
      }
      if (command == 'l') {
        motors_set_speed(motor_left, -RC_FULL_SPEED/speed_divisor);
        motors_set_speed(motor_right, -RC_TURNING_SPEED/speed_divisor);
      }
      if (command == 'm') {
        motors_set_speed(motor_left, -RC_TURNING_SPEED/speed_divisor);
        motors_set_speed(motor_right, -RC_FULL_SPEED/speed_divisor);
      }
      if (command == 'n') {
        motors_set_speed(motor_left, RC_TURNING_SPEED/speed_divisor);
        motors_set_speed(motor_right ,RC_FULL_SPEED/speed_divisor);
      }  
      if (command == 'c') {
        motors_set_speed(motor_left, -RC_FULL_SPEED/speed_divisor);
        motors_set_speed(motor_right, -RC_FULL_SPEED/speed_divisor);
      } 
      if (command == 'd') {
        motors_set_speed(motor_left, -RC_FULL_SPEED/speed_divisor);
        motors_set_speed(motor_right, RC_FULL_SPEED/speed_divisor);
      } 
      if (command == 'b') {
        motors_set_speed(motor_left, RC_FULL_SPEED/speed_divisor);
        motors_set_speed(motor_right, -RC_FULL_SPEED/speed_divisor);
      } 
      if (command == '0') {
        motors_set_speed(motor_left, 0);
        motors_set_speed(motor_right, 0);
      }
      if (command == 'C') {
        state = IDLE;
      } else {
        state = RC;
      }
      break;
    }
    case AUTONOMOUS: {
      if (command != 'C' && !force_rc_mode) {
        autonomous_mode_FSM();
        state = AUTONOMOUS;
      }
      else {
        state = RC;
      }
      break;
    }
  }
}
