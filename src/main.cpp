#include <Arduino.h>
#include "adc.h"
#include "battery.h"
#include "cli.h"
#include "config.h"
#include "encoders.h"
#include "maze.h"
#include "motion.h"
#include "motors.h"
#include "mouse.h"
#include "reporting.h"
#include "sensors.h"
#include "switches.h"
#include "systick.h"

Systick systick;                          // the main system control loop
Encoders encoders;                        // tracks the wheel encoder counts
AnalogueConverter adc;                    // controls all analogue conversions
Battery battery(BATTERY_ADC_CHANNEL);     // monitors battery voltage
Switches switches(SWITCHES_ADC_CHANNEL);  // monitors the button and switches
Sensors sensors;                          // make sensor alues from adc vdata
Motion motion;                            // high level motion operations
Motors motors;                            // low level control for drive motors
Profile forward;                          // speed profiles for forward motion
Profile rotation;                         // speed profiles for rotary motion
Maze maze PERSISTENT;                     // holds maze map (even after a reset)
Mouse mouse;                              // all the main robot logic is here
CommandLineInterface cli;                 // user interaction on the serial port
Reporter reporter;                        // formatted reporting of robot state

/******************************************************************************/

// The encoder ISR routines do not need to be explicitly defined
// because the callbacks are assigned though the Arduino API
// Other interrupt service routines are defined here for want of
// a better place to put them

ISR(ADC_vect) {
  adc.callback_adc_isr();
}

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}

/******************************************************************************/


// variables here for initial testing
int m_switches_adc;
float m_distance;
int m_front_sum;
int m_front_diff;

void setup() {
  Serial.begin(BAUDRATE);
  //redirectPrintf(); // send printf output to Serial (uses 20 bytes RAM)
  pinMode(LED_USER, OUTPUT);
  digitalWrite(LED_USER, 0);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);

  adc.begin();
  encoders.begin();
  motors.begin();
  systick.begin();

  if (switches.button_pressed()) {
    maze.initialise();
    mouse.blink(2);
    Serial.println(F("Maze cleared"));
    switches.wait_for_button_release();
  }
  /// leave the emitters off unless we are actually using the sensors
  /// less power, less risk
  sensors.disable();
  maze.set_goal(GOAL);
  reporter.set_printer(Serial);

  Serial.println();
  Serial.println(F(CODE));
  Serial.println(F(NAME));
  Serial.println(F("RDY"));
  cli.prompt();
}

void loop() {

  if (switches.button_pressed()) {
    switches.wait_for_button_release();
    int function = switches.read();
    cli.run_function(function);
  } else if (cli.read_serial()) {
    cli.interpret_line();
  }

//  m_switches_adc = switches.adc_reading();
// Serial.println(m_switches_adc);
//  delay(1000);

/*  
  sensors.enable();

  m_front_sum = sensors.get_front_sum();
  m_front_diff = sensors.get_front_diff();

  Serial.println("sensors");
  Serial.println(m_front_sum);
  Serial.println(m_front_diff);
  
  delay(1000);
*/

/*
  if (switches.button_pressed()) {
    switches.wait_for_button_release();
    int function = switches.read();
    Serial.println(function);
    cli.run_function(function);
  }
*/

/*
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 50);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 50);

  delay(2000);                       // wait for a second
   m_distance = encoders.robot_distance();
  Serial.println(m_distance);

  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, 50);
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 50);

  delay(2000);                       // wait for a second
  Serial.println(m_distance);
*/

 
}

