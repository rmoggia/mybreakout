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




#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>

#define WIFI_SSID "IoTDiPalma"
#define WIFI_PWD "IoTDiPalma@2020"
#define WIFI_CONNECT_DELAY 500
#define WIFI_START_DELAY 10

#define MQTT_SERVER "192.168.1.130"
#define MQTT_USER "mqtt_user"
#define MQTT_PWD "mqtt_user/1960"
#define MQTT_SERVER_PORT 1883

#define HUMIDITY_TOPIC "sensor/umiditaSalotto"
#define TEMPERATURE_TOPIC "sensor/temperaturaSalotto"
#define BRIGHTNESS_TOPIC "sensor/luminositaSalotto"

#define DHTTYPE DHT11
#define DHTPIN D5
#define DHTPARAM 11


#ifdef MUX true
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
#endif

#define ANALOG_INPUT A0

#define SERIAL_BAUD 9600

#define TICK_CONTROL 2000
#define TICK_RETRY_CONNECT 5000


/*
 * global variables
 */
long lastMsg_DHT = 0;
float temp_DHT = 0.0;
float hum_DHT = 0.0;
float diffTemp_DHT = 1.0;
float diffHumid_DHT = 1.0;
/*
 * 
 */
float diffLux_BH = 5.0;
float lux_BH = 0;/*
 * variables 
 */
// Set GPIOs for LED and PIR Motion Sensor
const int LED = 12;
const int MOTIONSENSOR = 14; 
// Timer: Auxiliary variables
unsigned long now_MT = millis();
unsigned long lastTrigger_MT = 0;
boolean startTimer_MT = false;
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
  dht.begin();
  client.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
  //
  #ifdef MUX
     setupMUX();
     #ifdef MUX_0
      setup_BH1750();   
     #endif
  #endif
}
/*
 * 
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
float value;
  #ifdef MUX_0 
    changeMux(LOW, LOW, LOW);
    value = analogRead(ANALOG_INPUT); //Value of the sensor connected Option 0 pin of Mux
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
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
  if (now - lastMsg_DHT > TICK_CONTROL) {
    lastMsg_DHT = now;

    float newTemp_DHT = dht.readTemperature();
    float newHum_DHT = dht.readHumidity();
    float newLux_BH = analogRead(A0);
   
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
    // luminosità 
    if (checkBound(newLux_BH, lux_BH, diffLux_BH)) {
     lux_BH = newLux_BH;
     Serial.print("Luminosita':");
     Serial.println(String(lux_BH).c_str());
     client.publish(BRIGHTNESS_TOPIC, String(lux_BH).c_str(), true);
    }
  }
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
  //Deifne output pins for Mux
  #ifdef MUX
    pinMode(MUX_A, OUTPUT);
    pinMode(MUX_B, OUTPUT);     
    pinMode(MUX_C, OUTPUT);  
  #endif 
  //
}
/*
 * 
 */
void setup_MotionDetect() {

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(MOTIONSENSOR, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(MOTIONSENSOR), detectsMovement, RISING);

  // Set LED to LOW
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
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
      Serial.print("failed, rc=");
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
// Checks if motion was detected, sets LED HIGH and starts a timer
ICACHE_RAM_ATTR void detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  digitalWrite(LED, HIGH);
  startTimer_MT = true;
  lastTrigger_MT = millis();
}
