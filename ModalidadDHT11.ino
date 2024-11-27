#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Stepper.h>

// Configuración Wi-Fi
const char* ssid = "Rex";
const char* password = "x8am9fqb";

// Configuración MQTT
const char* mqtt_server = "192.168.19.61";
const int mqtt_port = 1883;

// Configuración DHT11
#define DHTPIN 15
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Configuración LEDs
const int pinLedAzul = 12;
const int pinLedRojo = 13;

// Configuración Servo
Servo servo1, servo2;
const int servoPin1 = 25;
const int servoPin2 = 26;

// Configuración Stepper
#define IN1 32
#define IN2 33
#define IN3 27
#define IN4 14
const int stepsPerRevolution = 2048;
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// Variables de control
WiFiClient espClient;
PubSubClient client(espClient);
bool isAutomaticMode = false; // Modo inicial manual
const int motorSpeed = 18;
unsigned long lastTempCheck = 0;
unsigned long lastStepperRun = 0;
const unsigned long tempInterval = 2000;
const unsigned long stepperInterval = 120000;

// Función para configurar Wi-Fi
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

// Función para cosecha
void cosechar() {
  servo1.write(180);
  servo2.write(180);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
}

// Callback para mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String receivedTopic = String(topic);

  // Si el tópico es "modalidad"
  if (receivedTopic == "modalidad") {
    if (payload[0] == '1') { // Interpreta true como automático
      isAutomaticMode = true;
      Serial.println("Modo automático activado");
    } else if (payload[0] == '0') { // Interpreta false como manual
      isAutomaticMode = false;
      Serial.println("Modo manual activado");
    }
    return; // No procesar más lógica si se cambió el modo
  }

  // Lógica para el modo manual
  if (!isAutomaticMode) {
    String message;
    for (int i = 0; i < length; i++) message += (char)payload[i];
    
    if (message == "true" && receivedTopic == "motor/derecha") {
      myStepper.setSpeed(motorSpeed);
      myStepper.step(stepsPerRevolution);
    } else if (message == "true" && receivedTopic == "motor/izquierda") {
      myStepper.setSpeed(motorSpeed);
      myStepper.step(-stepsPerRevolution);
    } else if (message == "true" && receivedTopic == "garra") {
      cosechar();
    }
  }
}

// Reconexión al broker MQTT
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      client.subscribe("modalidad");
      client.subscribe("motor/derecha");
      client.subscribe("motor/izquierda");
      client.subscribe("garra");
    } else {
      delay(5000);
    }
  }
}

// Configuración inicial
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(pinLedAzul, OUTPUT);
  pinMode(pinLedRojo, OUTPUT);
  dht.begin();

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo1.write(90);
  servo2.write(90);

  myStepper.setSpeed(motorSpeed);
}

// Lógica principal
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // Lógica automática
  if (isAutomaticMode) {
    if (millis() - lastTempCheck > tempInterval) {
      lastTempCheck = millis();
      float temp = dht.readTemperature();
      float hum = dht.readHumidity();
      if (!isnan(temp) && !isnan(hum)) {
        digitalWrite(pinLedAzul, temp >= 10 && temp <= 35 ? HIGH : LOW);
        digitalWrite(pinLedRojo, temp < 10 || temp > 35 ? HIGH : LOW);
      }
    }

    if (millis() - lastStepperRun > stepperInterval) {
      lastStepperRun = millis();
      myStepper.step(-stepsPerRevolution * 18 * 2);
      delay(10000);
      myStepper.step(stepsPerRevolution * 18 * 2);
      delay(10000);
    }
  }
}
