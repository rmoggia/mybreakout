// Get ESP8266 going with Arduino IDE
// - https://github.com/esp8266/Arduino#installing-with-boards-manager
// Required libraries (sketch -> include library -> manage libraries)
// - PubSubClient by Nick ‘O Leary
// - DHT sensor library by Adafruit

//---- MAPPA PIN DIGITALI
// D0-D1-D2 ---> 74HC4051
// D3-D4    ---> BH1750
// D5       ---> DHT11
// D6       ---> HC-SR501
//---- MAPPA PIN ANOLOGICI 74HC4051
// A0       ---> MQ-5

#include <stdint.h> 
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>

const char* WIFI_SSID = "IoTDiPalma";
const char* WIFI_PWD = "IoTDiPalma@2020";
const uint16_t WIFI_CONNECT_DELAY = 500;
const uint8_t WIFI_START_DELAY = 10;

const char* MQTT_SERVER = "192.168.1.130";
const char* MQTT_USER = "mqtt_user";
const char* MQTT_PWD = "mqtt_user/1960";
const uint16_t MQTT_SERVER_PORT = 1883;

const char* HUMIDITY_TOPIC = "sensor/umiditaSalotto";
const char* TEMPERATURE_TOPIC = "sensor/temperaturaSalotto";
const char* BRIGHTNESS_TOPIC = "sensor/luminositaSalotto";
const char* PRESENCE_TOPIC = "sensor/presenzaSalotto";

const uint8_t DHTTYPE = DHT11;
const uint8_t DHTPIN = D5;
const uint8_t DHTPARAM = 11;


#define MUX_A D0
#define MUX_B D1
#define MUX_C D2

#define MUX_0 true    //BH1750
#define MUX_1 false
#define MUX_2 false
#define MUX_3 false
#define MUX_4 false
#define MUX_5 false
#define MUX_6 false
#define MUX_7 false
#define MUX_8 false


const unsigned short ANALOG_INPUT = A0;

const uint32_t SERIAL_BAUD = 9600;

const uint16_t TICK_CONTROL = 2000;
const uint16_t TICK_RETRY_CONNECT = 5000;

const uint8_t TIME_SECOND_MT = 10;


/*
 * global variables
 */
uint32_t lastMsg_DHT = 0;
float temp_DHT = 0.0;
float hum_DHT = 0.0;
float diffTemp_DHT = 1.0;
float diffHumid_DHT = 1.0;
/*
 * 
 */
float diffLux_BH = 5.0;
float lux_BH = 0;
/*
 * variables 
 */
// Set GPIOs for LED_MOTION_DETECT and PIR Motion Sensor
const unsigned short LED_MOTION_DETECT = 12;
const unsigned short MOTION_SENSOR_PIN = 14; 
// Timer: Auxiliary variables
unsigned long motionDetectLastTrigger = 0;
boolean motionDetectStartTimer = false;
//
WiFiClient espClient;
//
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE,DHTPARAM); // 11 works fine for ESP8266
BH1750 lightMeter;


/*
 * general setup
 */

void setup() {
  Serial.begin(SERIAL_BAUD);
  setup_MotionDetect();
  setup_wifi();
  setup_BH1750(); 
  dht.begin();
  client.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
  /*
  #ifdef MUX
     setupMUX();
     #ifdef MUX_0
      setup_BH1750();   
     #endif
  #endif
*/  
}

/*
 * setup wifi for mqtt
 */
void setup_wifi() {
  delay(WIFI_START_DELAY);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PWD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_CONNECT_DELAY);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
/*
 * general loop
 */
void loop() {
  long now = millis();
  loop_MotionDetect(now);
  loop_Mux();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  
  if (now - lastMsg_DHT > TICK_CONTROL) {
    lastMsg_DHT = now;

    float newTemp_DHT = dht.readTemperature();
    float newHum_DHT = dht.readHumidity();
   
    // temperatura
    if (checkBound(newTemp_DHT, temp_DHT, diffTemp_DHT)) {
      temp_DHT = newTemp_DHT;
      Serial.print("Temperatura:");
      Serial.println(String(temp_DHT).c_str());
      client.publish(TEMPERATURE_TOPIC, String(temp_DHT).c_str(), true);
    }
    // Umidita'
    if (checkBound(newHum_DHT, hum_DHT, diffHumid_DHT)) {
      hum_DHT = newHum_DHT;
      Serial.print("Umidita':");
      Serial.println(String(hum_DHT).c_str());
      client.publish(HUMIDITY_TOPIC, String(hum_DHT).c_str(), true);
    }
  }
}
/*
 * loop motion
 */
void loop_MotionDetect(unsigned long now) {
  // Current time
  now = now; //millis();
  // Turn off the LED_MOTION_DETECT after the number of seconds defined in the timeSeconds variable
  if(motionDetectStartTimer && (now - motionDetectLastTrigger > (TIME_SECOND_MT * 1000))) {
    Serial.println("Motion stopped...");
    //digitalWrite(LED_MOTION_DETECT, LOW);
    motionDetectStartTimer = false;
  }
}
/*
 * loop mux
 */
 void loop_Mux() {
  float value;
  #ifdef MUX_0 
    changeMux(LOW, LOW, LOW);
    float newLux_BH = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 0 pin of Mux
    // luminosità 
    if (checkBound(newLux_BH, lux_BH, diffLux_BH)) {
     lux_BH = newLux_BH;
     Serial.print("Luminosita':");
     Serial.println(String(lux_BH).c_str());
     client.publish(BRIGHTNESS_TOPIC, String(lux_BH).c_str(), true);
    }    
  #endif  
  
  #ifdef MUX_1 
    changeMux(LOW, LOW, HIGH);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 1 pin of Mux
  #endif  
    
  #ifdef MUX_2 
    changeMux(LOW, HIGH, LOW);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 2 pin of Mux
  #endif  
  
  #ifdef MUX_3   
    changeMux(LOW, HIGH, HIGH);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 3 pin of Mux
  #endif  
  
  #ifdef MUX_4   
    changeMux(HIGH, LOW, LOW);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 4 pin of Mux
  #endif  
  
  #ifdef MUX_5 
    changeMux(HIGH, LOW, HIGH);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 5 pin of Mux
  #endif  
  
  #ifdef MUX_6 
    changeMux(HIGH, HIGH, LOW);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 6 pin of Mux
  #endif  
  
  #ifdef MUX_7 
    changeMux(HIGH, HIGH, HIGH);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 7 pin of Mux
  #endif  
  //
}
/*
 * 
 */
bool setup_BH1750()  {

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  //Wire.begin();
  // On esp8266 you can select SCL and SDA pins using 
  Wire.begin(D4, D3);
  if (lightMeter.begin()) Serial.println(F("BH1750 initialised")); 
  else Serial.println(F("Error initialising BH1750")); 
  Serial.println(F("BH1750 Test begin"));
}
/*
 * 
 */
void setup_MUX() {
  
  //define output pins for Mux
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);     
  pinMode(MUX_C, OUTPUT);  
}
/*
 * 
 */
void setup_MotionDetect() {

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(MOTION_SENSOR_PIN, INPUT_PULLUP);
  // Set MOTION_SENSOR_PIN pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), detectsMovement, RISING);

  // Set LED_MOTION_DETECT to LOW
  pinMode(LED_MOTION_DETECT, OUTPUT);
  digitalWrite(LED_MOTION_DETECT, LOW);
}

/*
 * 
 */
void changeMux(int c, int b, int a) {
  #ifdef MUX
    digitalWrite(MUX_A, a);
    digitalWrite(MUX_B, b);
    digitalWrite(MUX_C, c);
  #endif
}
/*
 * 
 */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect("ESP8266Client", MQTT_USER, MQTT_PWD)) {
      Serial.println("connected");
    } else {
      Serial.print("faiLED_MOTION_DETECT, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(TICK_RETRY_CONNECT);
    }
  }
}
/*
 * 
 */
bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}
/*
 * 
 */
// Checks if motion was detected, sets LED_MOTION_DETECT HIGH and starts a timer
ICACHE_RAM_ATTR void detectsMovement() {
  //Serial.println("MOTION DETECTED!!!");//non consigliato
  digitalWrite(LED_MOTION_DETECT, HIGH);
  motionDetectStartTimer = true;
  motionDetectLastTrigger = millis();
}
