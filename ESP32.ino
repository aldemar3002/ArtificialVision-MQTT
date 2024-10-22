#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>

// Configuración Wi-Fi
const char* ssid = "Park-P12-Depa7";
const char* password = "ParkLifeP12-d7";

// Configuración MQTT
const char* mqtt_server = "172.18.0.92";  // Dirección IP del broker MQTT
const int mqtt_port = 1883;

// Configuración DHT11
#define DHTPIN 13    // Pin donde está conectado el sensor DHT11
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Configuración del LED
const int ledPin = 2; // Pin donde está conectado el LED

// Configuración de Servomotores
Servo servo1;  // Primer servomotor
Servo servo2;  // Segundo servomotor
const int servoPin1 = 25;  // Pin para el primer servo
const int servoPin2 = 26;  // Pin para el segundo servo

WiFiClient espClient;
PubSubClient client(espClient);

// Función de conexión a Wi-Fi
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

// Función de reconexión MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client")) {  // Conexión sin usuario ni contraseña
      Serial.println("conectado");
      client.subscribe("esp32/LED");        // Suscripción para controlar el LED
      client.subscribe("servos/sensor");  // Suscripción para el detector de jitomates
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Intenta de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

// Función para cosechar jitomates
void cosechar() {
  // Mover servo1 y servo2 a la posición deseada para cosechar
  servo1.write(180);  // Ángulo de trabajo
  servo2.write(180);  // Ángulo de trabajo
  delay(1000);        // Espera un segundo (simula el tiempo de cosecha)
  
  // Después de la cosecha, regresa los servos a una posición intermedia o de reposo
  servo1.write(90);   // Posición de reposo (ajusta este valor según lo necesites)
  servo2.write(90);   // Posición de reposo
  delay(1000);        // Espera un segundo para finalizar
}


// Función callback para manejar mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "esp32/LED") {
    if (message == "true") {
      digitalWrite(ledPin, HIGH); // Enciende el LED
    } else if (message == "false") {
      digitalWrite(ledPin, LOW); // Apaga el LED
    }
  }

  if (String(topic) == "servos/sensor") {
    if (message == "cosechar") {
      cosechar(); // Activar los servos para la cosecha
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); // Configurar la función de callback

  pinMode(ledPin, OUTPUT); // Configurar el pin del LED como salida
  dht.begin(); // Iniciar el sensor DHT11
  
  // Inicializar servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  
  // Posición inicial de los servos
  servo1.write(0);
  servo2.write(0);
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
    String message = String("Temp: ") + temperature + "C Humidity: " + humidity + "%";
    client.publish("laptop/aldemar", message.c_str()); // Publicar datos del DHT11
  }

  delay(2000); // Esperar 2 segundos antes de la siguiente lectura y publicación
}
