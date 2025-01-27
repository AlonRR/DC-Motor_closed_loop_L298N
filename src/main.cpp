/**
 * Project includes:
 * - L298N Dual H-Bridge Motor Controller
 * - DC Motor
 */
#include <Arduino.h>

#define CHANNEL 1
#define BRIDGE_PWM_PIN 12
#define BRIDGE_L1_PIN 33
#define BRIDGE_L2_PIN 25
#define MOTOR_PIN_C1 26
#define MOTOR_PIN_C2 27

int counter = 0;

void counter_ISR()
{
    digitalRead(MOTOR_PIN_C1) == digitalRead(MOTOR_PIN_C2) ? counter-- : counter++;
}

void setup()
{
    Serial.begin(115200);
    // Set the Bridge pins as output
    pinMode(BRIDGE_L1_PIN, OUTPUT);
    pinMode(BRIDGE_L2_PIN, OUTPUT);
    pinMode(BRIDGE_PWM_PIN, OUTPUT);

    // Set the Motor pins as input
    pinMode(MOTOR_PIN_C1, INPUT);
    pinMode(MOTOR_PIN_C2, INPUT);

    // Attach the interrupt for the motor
    attachInterrupt(digitalPinToInterrupt(MOTOR_PIN_C1), counter_ISR, CHANGE);

    // Set the PWM for the motor
    /**
     *   uint32_t ledcSetup(
     *                  uint8_t channel,
     *                  uint32_t freq,
     *                  uint8_t resolution_bits)
     *
     channel 0-15, resolution 1-16bits, freq limits depend on resolution.
     */

    ledcSetup(CHANNEL, 5000, 8);
    ledcAttachPin(BRIDGE_PWM_PIN, CHANNEL);
}

void loop()
{
    int speed = 0;
    // Set the direction of the motor
    digitalWrite(BRIDGE_L1_PIN, HIGH);
    digitalWrite(BRIDGE_L2_PIN, LOW);
    for (int i = 255; i > 130; i--)
    {
        ledcWrite(CHANNEL, i);
        delay(10);
    }
    Serial.println(counter);
    digitalWrite(BRIDGE_L1_PIN, LOW);
    digitalWrite(BRIDGE_L2_PIN, HIGH);

    while (counter > 0)
    {
        speed = counter;
        if (counter < 500)
            speed = counter / 2;
        if (speed < 150)
            speed = 150;
        ledcWrite(CHANNEL, speed);
    }
    Serial.println(counter);
}
