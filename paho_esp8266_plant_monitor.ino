/*******************************************************************************
 *
 * ESP8266 MQTT Plant Moisture & Environment Monitor
 * An Arduino sketch for the ESP8266, Checks the Chirp moisture sensor and DHT11
 * every 15 seconds and publishes the data to an MQTT Server.
 *
 * Dependancies:
 *     - ESP8266 Arduino Core (Board Definition) https://github.com/esp8266/Arduino
 *     - DHT Library
 *     - MQTTClient Library (Eclipse Paho Wrapper) https://github.com/256dpi/arduino-mqtt
 *
 * Contributors:
 *     - James Sutton - initial contribution
 *
 * Notes:
 *     - Haven't yet been able to get the pure Paho embedded-c library working
 *       on the ESP8266 yet so am currently using the wrapper.
 *
 *******************************************************************************/
#include <ESP8266WiFi.h>
#include <MQTTClient.h>
#include <DHT.h>
#include <Wire.h>

// WiFi
const char* ssid = "wirelessSSID";
const char* pass = "wirelessPASS";

// MQTT
const char* mqtt_server = "iot.eclipse.org";
//const char* mqtt_user = "username";
//const char* mqtt_pass = "password";

// Topics
char* tempTopic = "/home/temp";
char* humTopic = "/home/humidity";
char* plantTopic = "/home/plant";

#define DHTPIN 2
#define DHTTYPE DHT11  // DHT 11
#define CHIRPRESETPIN 16

WiFiClient net;
MQTTClient client;
DHT dht(DHTPIN, DHTTYPE, 30);

unsigned long lastUpdate = 0;

void connect(); // <- predefine connect() for setup()

// Converts a mac address to a String
String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}


void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(CHIRPRESETPIN, OUTPUT);  // Initialize the CHIRPRESETPIN as an output
  Serial.begin(115200);
  Wire.begin();
  client.begin(mqtt_server, net);
  setup_wifi();
  connect();

}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void connect(){
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  String clientName;
  clientName += "esp8266-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);

  Serial.print("Connecting to ");
  Serial.print(mqtt_server);
  Serial.print(" as ");
  Serial.println(clientName);

  // Use this if using username and passwords
  //while(!client.connect((char*) clientName.c_str(), (char*) mqtt_user, (char*) mqtt_pass)){
  while(!client.connect((char*) clientName.c_str())){
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
    Serial.print("Connection failed, rc=?");
    //Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 2 seconds before retrying
    delay(2000);
  }
 Serial.println("Successfully connected.");
}

void loop() {
  client.loop();
  delay(10); // Fixes some issues with the WiFi

  if(!client.connected()) {
   connect();
  }

  // Publish a message roughly every 15 seconds
  if(millis() - lastUpdate > 15000) {
   lastUpdate = millis();
   publishUpdate();
  }
}

boolean publishUpdate(){
   Serial.println("Publishing update.");
   // Publish an update
   // Get Plant Pot Moisture Value
    int moistureVal = getMoistureReading();
    String plantPayload = "{\"_type\":\"moisture\",\"value\":";
    plantPayload += moistureVal;
    plantPayload += "}";

    Serial.print("Plant moisture payload: ");
    Serial.println(plantPayload);
    client.publish(plantTopic, (char*) plantPayload.c_str());

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      return false;

    } else {
      String tempPayload = "{\"_type\":\"temp\",\"value\":";
      tempPayload += t;
      tempPayload += "}";

      String humPayload = "{\"_type\":\"humidity\",\"value\":";
      humPayload += h;
      humPayload += "}";

      Serial.print("Sending temperature payload: ");
      Serial.println(tempPayload);
      client.publish(tempTopic, (char*) tempPayload.c_str());

      Serial.print("Sending humidity payload: ");
      Serial.println(humPayload);
      client.publish(humTopic, (char*) humPayload.c_str());
      Serial.println("--------------------------");
      return true;
    }
}


unsigned int readI2CRegister16bit(int addr, int reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(1100);
  Wire.requestFrom(addr, 2);
  unsigned int t = Wire.read() << 8;
  t = t | Wire.read();
  return t;
}

unsigned int getMoistureReading(){
  int moistureVal = readI2CRegister16bit(0x20, 0);
  while(moistureVal == -1){
    delay(500);
    Serial.println("Bad reading from chirp, resetting...");
    // Chirp is not responding, reset
    // Set Chirp reset pin to high
    digitalWrite(CHIRPRESETPIN, LOW);
    delay(500);
    digitalWrite(CHIRPRESETPIN, HIGH);
    delay(500);
    moistureVal = readI2CRegister16bit(0x20, 0);
  }

  while(moistureVal == 65535){
    moistureVal = readI2CRegister16bit(0x20, 0);
  }
  return moistureVal;
}


void messageReceived(String topic, String payload, char * bytes, unsigned int length) {
  Serial.print("incoming: ");
  Serial.print(topic);
  Serial.print(" - ");
  Serial.print(payload);
  Serial.println();
}
