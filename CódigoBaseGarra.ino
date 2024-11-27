#include <ESP32Servo.h>

// Pin al que conectas el servo
int servoPin = 18;

// Crear el objeto servo
Servo myServo;

void setup() {
  // Iniciar la comunicación serial para monitorear
  Serial.begin(115200);

  // Configurar el pin del servo
  myServo.attach(servoPin);

  // Mover el servo a la posición inicial
  myServo.write(0);
  Serial.println("Servo en posición inicial: 0°");
  delay(1000);
}

void loop() {
  // Mover el servo a 180°
  Serial.println("Moviendo a 180°");
  myServo.write(0);
  delay(1000);

  // Mover el servo a 0°
  Serial.println("Moviendo a 0°");
  myServo.write(180);
  delay(1000);
}
