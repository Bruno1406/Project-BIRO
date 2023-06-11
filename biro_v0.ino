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
#define MOTOR_LEFT_DIRECTION_PIN 8
 
#define MOTOR_RIGHT_PWM_PIN 10
#define MOTOR_RIGHT_DIRECTION_PIN 7

#define MAX_SPEED 100
#define MIN_SPEED -100

#define RC_FULL_SPEED 80
#define FOLLOW_LINE_SPEED 30
#define CROSS_H_TURNING_SPEED 70
#define CROSS_H_STRAIGHT_SPEED 50

// SERVO MOTOR
#define SERVO_PIN 6

#define BARRIER_UP_ANGLE 90
#define BARRIER_DOWN_ANGLE 20

// DISTANCE SENSORS
#define DS_LEFT_ECHO_PIN 2
#define DS_LEFT_TRIGGER_PIN 3

#define DS_RIGHT_ECHO_PIN 0
#define DS_RIGHT_TRIGGER_PIN 1

#define DISTANCE_SENSORS_THRESHOLD 30

// LINE SENSORS
#define LS_FAR_LEFT_READ_PIN A0
#define LS_CENTER_LEFT_READ_PIN A1
#define LS_CENTER_RIGHT_READ_PIN A2
#define LS_FAR_RIGHT_READ_PIN A3

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
#define GRAB_OBJECT_ROUTINE_START 10000
#define GRAB_OBJECT_ROUTINE_END 120000
#define CROSS_H_DECELERATION 50
#define CROSS_H_TURNING CROSS_H_DECELERATION + 300
#define CROSS_H_STRAIGHT_LINE CROSS_H_TURNING + CROSS_H_DECELERATION + 2000


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
  main_FSM();   
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
  num_of_readings = (hcsr04_get_distance(ds_left) > DISTANCE_SENSORS_THRESHOLD || hcsr04_get_distance(ds_right) > DISTANCE_SENSORS_THRESHOLD) ?
                    num_of_readings++ : 0;
  return num_of_readings > 5; 
}

bool TCR5000_is_on_line(line_sensors_t TCR5000_sensor) {
  return (analogRead(TCR5000_sensor.reading_adc_pin) > LINE_SENSORS_THRESHOLD);
}

char bluetooth_get_command() {
  char command = '0';
  if (bt.available()) {
    command = bt.read();
  }
  if (Serial.available()) {
      bt.write(Serial.read());
  }
  return command;
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
  uint32_t time = millis() - start_time;
  switch (state) {
    case WAIT_FOR_START: {
      if (command == 'S') {
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
      if (hcsr04_is_not_seeing_wall()) {
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
      if (command == 'X' && time == 0) {
        state = AUTONOMOUS;
        time = millis();
      } else if (command == 'X' && time != 0) {
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
      if (command == 'C') {
          is_full_speed = ~is_full_speed;
      }
      if (command == 'S') {
        angle = angle != BARRIER_UP_ANGLE ? BARRIER_UP_ANGLE : BARRIER_DOWN_ANGLE;
        servo_set_angle(barrier_servo, angle);
      }
      if (command == 'U') {
          motors_set_speed(motor_left, RC_FULL_SPEED/speed_divisor);
          motors_set_speed(motor_right, RC_FULL_SPEED/speed_divisor);
      } 
      if (command == 'D') {
          motors_set_speed(motor_left, -RC_FULL_SPEED/speed_divisor);
          motors_set_speed(motor_right, -RC_FULL_SPEED/speed_divisor);
      } 
      if (command == 'L') {
          motors_set_speed(motor_left, -RC_FULL_SPEED/speed_divisor);
          motors_set_speed(motor_right, RC_FULL_SPEED/speed_divisor);
      } 
      if (command == 'R') {
          motors_set_speed(motor_left, RC_FULL_SPEED/speed_divisor);
          motors_set_speed(motor_right, -RC_FULL_SPEED/speed_divisor);
      } 
      if (command == '0') {
          motors_set_speed(motor_left, 0);
          motors_set_speed(motor_right, 0);
      }
      if (command == 'X') {
        state = IDLE;
      } else {
        state = RC;
      }
      break;
    }
    case AUTONOMOUS: {
      if (command != 'X') {
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
