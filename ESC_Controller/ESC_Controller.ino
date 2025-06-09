#include <Servo.h>

#define ESC_PIN 6  // ESC connected to GP6

Servo esc;
int throttle = 0;  // Stores throttle value
bool motorOn = false;  // Motor state

void setup() {
  Serial.begin(115200);
  esc.attach(ESC_PIN);

  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1000);  // Min throttle for arming
  delay(3000);

  Serial.println("Press 'S' once to switch on, type in your throttle, and press 'S' again to switch off");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove extra spaces/newlines

    int newThrottle = input.toInt();
      if (newThrottle >= 0 && newThrottle <= 100) {
        throttle = newThrottle;
        Serial.print("Throttle set to ");
        Serial.print(throttle);
        Serial.println("%");
      }

    if (input.equalsIgnoreCase("S")) {
      motorOn = true;;
    } 
  }

  // Only send throttle if motor is ON
  int pulse = motorOn ? map(throttle, 0, 100, 1000, 2000) : 1000;
  esc.writeMicroseconds(pulse);

  delay(100);
}
