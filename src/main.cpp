#include <Arduino.h>
#include "config.h"
#include "encoders.h"
#include "systick.h"

Systick systick;                          // the main system control loop
Encoders encoders;                        // tracks the wheel encoder counts

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}

float m_distance;

void setup() {
  Serial.begin(BAUDRATE);
  redirectPrintf(); // send printf output to Serial (uses 20 bytes RAM)
  //pinMode(LED_USER, OUTPUT);
  //digitalWrite(LED_USER, 0);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);

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


 
}

