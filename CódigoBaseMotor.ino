#include <Stepper.h>

// Número de pasos por revolución del 28BYJ-48 (con reductor interno)
const int stepsPerRevolution = 2048;

// Pines conectados al ULN2003
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26

// Inicializar el objeto Stepper

Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

void setup() {
  // Configurar velocidad del motor (en RPM)
  myStepper.setSpeed(18); // Velocidad en revoluciones por minuto (RPM)

  // Inicializar el puerto serie para monitoreo
  Serial.begin(115200);
  Serial.println("Motor configurado para ciclos de rotación");
}

void loop() {
  // Calcular el número total de pasos para 2 minutos
  int stepsForTwoMinutes = 18 * stepsPerRevolution * 2; // 18 RPM × 2048 pasos/rev × 2 minutos

  // Giro antihorario
  Serial.println("Giro antihorario por 2 minutos");
  myStepper.step(-stepsForTwoMinutes); // Rotación en sentido antihorario

  // Pausa de 10 segundos
  delay(1000);

  // Giro horario
  Serial.println("Giro horario por 2 minutos");
  myStepper.step(stepsForTwoMinutes); // Rotación en sentido horario

  // Pausa de 10 segundos antes de repetir
  delay(1000);
}
