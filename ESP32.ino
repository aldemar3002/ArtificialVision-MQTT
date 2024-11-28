#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Stepper.h>

// Configuración Wi-Fi
const char* ssid = "AldemariPhone";
const char* password = "perrosalchicha";

// Configuración MQTT
const char* mqtt_server = "172.20.10.10";  // Dirección IP del broker MQTT
const int mqtt_port = 1883;

// Configuración DHT11
#define DHTPIN 15    // Pin donde está conectado el sensor DHT11
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Configuración de LEDs adicionales
const int pinLedAzul = 12; // LED azul
const int pinLedRojo = 13; // LED rojo

// Configuración de Servomotores
Servo servo1;  // Primer servomotor
const int servoPin1 = 18;  // Pin para el primer servo

// Pines del motor paso a paso y configuración
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26
const int stepsPerRevolution = 2048;

// Inicializar el motor paso a paso
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// Configuración de velocidad del motor (en RPM)
const int motorSpeed = 18;

// Variables para MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Función para configurar Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
}

// Función para mover la garra con lógica no bloqueante
void moverGarra() {
  static unsigned long lastTime = 0;
  static bool isOpening = true;
  if (millis() - lastTime > 5000) {
    if (isOpening) {
      Serial.println("Abriendo garra (180°)...");
      servo1.write(180);
    } else {
      Serial.println("Cerrando garra (0°)...");
      servo1.write(0);
    }
    isOpening = !isOpening;
    lastTime = millis();
  }
}

// Función para manejar mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "motor/derecha" && message == "true") {
    Serial.println("Recibido: motor/derecha TRUE");
    myStepper.setSpeed(motorSpeed);
    myStepper.step(stepsPerRevolution);  // Giro horario
  } else if (String(topic) == "motor/izquierda" && message == "true") {
    Serial.println("Recibido: motor/izquierda TRUE");
    myStepper.setSpeed(motorSpeed);
    myStepper.step(-stepsPerRevolution); // Giro antihorario
  } else if (String(topic) == "motor/garra" && message == "true") {
    Serial.println("Recibido: garra TRUE");
    moverGarra();
  } else {
    Serial.println("Mensaje o tema desconocido.");
  }
}

// Función para reconectar al broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client")) {  // Conexión sin usuario ni contraseña
      Serial.println("Conectado al broker");
      client.subscribe("motor/derecha");
      client.subscribe("motor/izquierda");
      client.subscribe("motor/garra");
      client.subscribe("laptop/tomate/x");  // Ejemplo adicional
      client.subscribe("laptop/cuadrado/x");
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configuración Wi-Fi
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Inicializar el sensor DHT
  dht.begin();

  // Configurar pines de LEDs
  pinMode(pinLedAzul, OUTPUT);
  pinMode(pinLedRojo, OUTPUT);

  // Inicializar el servomotor
  servo1.attach(servoPin1);

  // Configuración del motor paso a paso
  myStepper.setSpeed(motorSpeed);

  Serial.println("Setup completo");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer datos del sensor DHT11
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (!isnan(humidity) && !isnan(temperature)) {
    // Crear un mensaje para publicar los datos
    String message = String() + temperature + "°C";
    String message2 = String() + humidity + "%";
    client.publish("laptop/temperatura", message.c_str()); // Publicar datos del DHT11
    client.publish("laptop/humedad", message2.c_str());

    // Control de LEDs según la temperatura
    if (temperature >= 10 && temperature <= 35) {
      digitalWrite(pinLedAzul, HIGH);  // Encender LED azul
      digitalWrite(pinLedRojo, LOW);  // Apagar LED rojo
    } else {
      digitalWrite(pinLedAzul, LOW);  // Apagar LED azul
      digitalWrite(pinLedRojo, HIGH); // Encender LED rojo
    }
  } else {
    Serial.println("Error leyendo el sensor DHT11.");
  }

  delay(2000); // Esperar 2 segundos antes de la siguiente lectura y publicación
}
