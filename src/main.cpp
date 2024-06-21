#include <Arduino.h>
#include "adc.h"
#include "config.h"
#include "encoders.h"
#include "sensors.h"
#include "switches.h"
#include "systick.h"

Systick systick;                          // the main system control loop
Encoders encoders;                        // tracks the wheel encoder counts
AnalogueConverter adc;                    // controls all analogue conversions
Switches switches(SWITCHES_ADC_CHANNEL);  // monitors the button and switches
Sensors sensors;                          // make sensor alues from adc vdata

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

void setup() {
  Serial.begin(BAUDRATE);
  //redirectPrintf(); // send printf output to Serial (uses 20 bytes RAM)
  pinMode(LED_USER, OUTPUT);
  digitalWrite(LED_USER, 1);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);

  adc.begin();
  encoders.begin();
  systick.begin();

  Serial.println();
  Serial.println(F(CODE));
  Serial.println(F(NAME));
  Serial.println(F("RDY"));
  m_distance = encoders.robot_distance();
  Serial.println(m_distance);

}

void loop() {

//  m_switches_adc = switches.adc_reading();
// Serial.println(m_switches_adc);
//  delay(1000);
  


  if (switches.button_pressed()) {
    switches.wait_for_button_release();
    int function = switches.read();
    Serial.println(function);
  }


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

