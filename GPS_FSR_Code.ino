#include <ESP8266WiFi.h>
#include <PubSubClient.h>     //MQTT

#include <Arduino.h>          //GPS
#include <TinyGPS++.h>        //GPS
#include <SoftwareSerial.h>   //GPS

#define DEVICE ""

const char* ssid = "xxxxxx";                   // your wifi ssid
const char* password =  "xxxxxx";              // your wifi password
const char* mqttServer = "test.mosquitto.org";     // IP adress MQTT Broker
const int mqttPort = 1883;
const char* mqttUser = "mqtt1";      // if you don't have MQTT Username, no need input
const char* mqttPassword = "mqtt1";  // if you don't have MQTT Password, no need input

TinyGPSPlus gps;                            //GPS
SoftwareSerial gpsSerial(D7, D8);  // RX,TX //GPS
char buffer[100];                           //GPS

int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
   
 
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

void GPS() {
  
  Serial.println("debug");
    if (gps.location.isUpdated()) {
        
        double lat = gps.location.lat();
        double lng = gps.location.lng();
   
        char msg[100];
        char val[100];
        dtostrf(gps.location.lat(), 3, 6, val);
        sprintf(msg, "%s", val);
        Serial.println(gps.location.lat());
        client.publish("GPSlat", msg);
        
        char msg1[100];
        char val1[100];
        dtostrf(gps.location.lng(), 3, 6, val1);
        sprintf(msg1, "%s", val1);
        Serial.println(gps.location.lng());
        client.publish("GPSlng", msg1);
               
          snprintf(buffer, sizeof(buffer),  
                  "%.6f %.6f",
                  lat, lng);        
  }
}

void FSR(){
  fsrReading = analogRead(fsrPin);  
 
  Serial.print("Analog reading = ");
  Serial.print(fsrReading);     // the raw analog reading
  
    char msg[30];
    char val[10];
    dtostrf(fsrReading, 5, 0, val);
    sprintf(msg, "%s", val);
    client.publish("ForceSensingResistance", msg); 
 
  if (fsrReading < 10) {
     client.publish("FSR1", "ไม่มีคนนั้งเบาะ"); 
    } else if (fsrReading > 500) {
     client.publish("FSR1", "มีคนนั้งเบาะ");
  }
}

void loop() {
  
  if (!client.connected()){
    reconnect();
    }
   
    client.loop();
    while (gpsSerial.available() > 0) {
      
        if (gps.encode(gpsSerial.read())) {
                 
            GPS();
            FSR();
                      
        }
    }          
}
