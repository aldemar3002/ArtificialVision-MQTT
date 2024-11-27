#include <WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <ESP32Servo.h>

// Configuración Wi-Fi
const char* ssid = "";
const char* password = "";

// Configuración MQTT
//const char* mqtt_server = "";  // Dirección IP del broker MQTT
//const int mqtt_port = 1883;

// Pin del sensor DHT11
int pinDHT = 15;

// Pines para LEDs
const int pinLedAzul = 12; // LED azul
const int pinLedRojo = 13; // LED rojo

// Instancia del DHT
DHTesp dht;

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
      client.subscribe("esp32/LED");      // Suscripción para controlar el LED
      client.subscribe("servos/sensor"); // Suscripción para el detector de jitomates
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Intenta de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  dht.setup(pinDHT, DHTesp::DHT11);

  // Configuración de pines de LEDs como salida
  pinMode(pinLedAzul, OUTPUT);
  pinMode(pinLedRojo, OUTPUT);

  //setup_wifi();
  //client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  //if (!client.connected()) {
  //  reconnect();
  //}
  //client.loop();

  // Obtener datos de temperatura y humedad
  TempAndHumidity data = dht.getTempAndHumidity();

  if (isnan(data.temperature) || isnan(data.humidity)) {
    Serial.println("Error leyendo el sensor. Verifica conexiones.");
  } else {
    Serial.println("Temperatura: " + String(data.temperature, 2) + "°C");
    Serial.println("Humedad: " + String(data.humidity, 1) + "%");
    Serial.println("---");

    // Control de LEDs según la temperatura
    if (data.temperature >= 10 && data.temperature <= 35) {
      digitalWrite(pinLedAzul, HIGH);  // Encender LED azul
      digitalWrite(pinLedRojo, LOW);  // Apagar LED rojo
    } else {
      digitalWrite(pinLedAzul, LOW);  // Apagar LED azul
      digitalWrite(pinLedRojo, HIGH); // Encender LED rojo
    }
  }

  delay(1000); // Esperar 1 segundo
}
