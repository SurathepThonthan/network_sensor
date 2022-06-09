#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define DEVICE ""

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;      //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];         //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;             //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

const char* ssid = "xxxxxx";                   // your wifi ssid
const char* password =  "xxxxxx";              // your wifi password
const char* mqttServer = "test.mosquitto.org";    // IP adress Raspberry Pi
const int mqttPort = 1883;
const char* mqttUser = "mqtt1";      // if you don't have MQTT Username, no need input
const char* mqttPassword = "mqtt1";  // if you don't have MQTT Password, no need input

WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(115200);
  //Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((int)payload[i]);
  }

  Serial.println();
  Serial.println("-----------------------");

}

void reconnect() {
  //
  while (!client.connected()){
    Serial.print("Attempting MQTT connection...");
    //
    if(client.connect(DEVICE)) {
      Serial.println("connected");
      //
      //
      client.subscribe("response");
      } else{
        Serial.print("failled, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        //
        delay(5000);
        }
     }
  }
  
void loop()
{
  if (!client.connected()){
    reconnect();
    }
          
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
 

  Serial.print(irValue);
    char msg[30];
    char val[10];
    dtostrf(irValue, 5, 0, val);
    sprintf(msg, "%s", val);
    client.publish("irValue", msg); 
       
  Serial.print("\t");
  
  Serial.print(beatsPerMinute);
    char msg3[40];
    char val3[20];
    dtostrf(beatsPerMinute, 5, 2, val3);
    sprintf(msg3, "%s", val3);
    if (irValue < 50000){
        client.publish("beatsPerMinute", "0");
    }
    else if (irValue > 50000){
        client.publish("beatsPerMinute", msg3);
     }
    
  Serial.print("\t");
   
  Serial.print(beatAvg);
    char msg2[50];
    char val2[30];
    dtostrf(beatAvg, 3, 0, val2);
    sprintf(msg2, "%s", val2);
    if (irValue < 50000){
         client.publish("beatAvg", "0");
    }
    else if (irValue > 50000){
         client.publish("beatAvg", msg2);
    }
     if (irValue < 50000){
         Serial.print(" No finger?");
         client.publish("state sensor", "No finger?");
     } 
     else if (irValue > 50000){
         Serial.print("have finger");
         client.publish("state sensor", "have finger");
       
        }
        
  Serial.println();
        
}
