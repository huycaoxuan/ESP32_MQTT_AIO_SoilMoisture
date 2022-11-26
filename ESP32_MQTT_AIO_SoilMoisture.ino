/**
 * Ghi du lieu tu cam bien do am dat, su dung ESP32 - mod by HuyCX
 * lib: Wifimanager, AIO, DHT
 * Implements TRIGGEN_PIN button press, press for ondemand configportal, hold for 3 seconds for reset settings.
 */
#include <WiFiManager.h>
#include <WiFi.h> //Thu vien ESP32
//#include <ESP8266WebServer.h>
//#include "DHT.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "secrets.h"

// set pin numbers
const int TRIGGER_PIN = 4;  //Nut bam setup wifi
const int S_LED =  16;    // // LED bao trang thai
const int SoilPin = 34; // SoilMoisture sensor is connected to GPIO 34 (Analog ADC1_CH6) 
const int Relay1 = 17; // the on off button feed turns this Relay1 on/off

#define MQTT_UPDATE_INTERVAL 600000 //Thoi gian moi lan update len cloud mili giay
int soil_moisture1 = 0 ; //soil moisture
bool wifiok = false;     //Wifi status
//bool MQTT_status = false; //MQTT status
bool auto_mode = false;
uint16_t sliderval;

//String dataString;
//char charBuf[100];
//const unsigned long mqtt_interval = 5000; //Thoi gian ket noi lai MQTT
//unsigned long previousTime = 0;
//bool retries_status = false;
unsigned long   lastPub = 0;

// wifimanager can run in a blocking mode or a non blocking mode
// Be sure to know how to process loops with no delay() if using non blocking
bool wm_nonblocking = false; // change to true to use non blocking

WiFiManager wm; // global wm instance
WiFiManagerParameter custom_field; // global param ( for non blocking w params )

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient   wifiClient;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;


// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&wifiClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed for publishing.
Adafruit_MQTT_Publish soil = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil"); //check
/*
Adafruit_MQTT_Publish hum = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish soil2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil2");
Adafruit_MQTT_Publish hum2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity2");
Adafruit_MQTT_Publish soil3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil3");
Adafruit_MQTT_Publish hum3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity3");
*/
Adafruit_MQTT_Publish relay_status = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/relay-status");
//Sub
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/switch1");
Adafruit_MQTT_Subscribe slider = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/slider1"); // for setting the threshold of automatic turn on valve
//Can them mot subcribe de upload status

void MQTT_connect();

void setup() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  
  Serial.begin(115200);
  Serial.setDebugOutput(true);  
  delay(3000);
  Serial.println("\n Starting");
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(Relay1, OUTPUT);     // Initialize the Relay1 pin as an output
  pinMode(S_LED, OUTPUT);
  digitalWrite(Relay1, LOW);
  // wm.resetSettings(); // wipe settings

  if(wm_nonblocking) wm.setConfigPortalBlocking(false);

  // add a custom input field
  int customFieldLength = 40;
  
  // test custom html(radio)
  const char* custom_radio_str = "<br/><label for='customfieldid'>Custom Field Label</label><input type='radio' name='customfieldid' value='1' checked> One<br><input type='radio' name='customfieldid' value='2'> Two<br><input type='radio' name='customfieldid' value='3'> Three";
  new (&custom_field) WiFiManagerParameter(custom_radio_str); // custom html input
  
  wm.addParameter(&custom_field);
  wm.setSaveParamsCallback(saveParamCallback);

  // custom menu via array or vector
  // 
  // menu tokens, "wifi","wifinoscan","info","param","close","sep","erase","restart","exit" (sep is seperator) (if param is in menu, params will not show up in wifi page!)
  // const char* menu[] = {"wifi","info","param","sep","restart","exit"}; 
  // wm.setMenu(menu,6);
  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  // set dark theme
  wm.setClass("invert");

  // wm.setConnectTimeout(20); // how long to try to connect for before continuing
  wm.setConfigPortalTimeout(30); // auto close configportal after n seconds

  bool res;
  res = wm.autoConnect(WIFI_SSID01,WIFI_PW); // password protected ap

  if(!res) {
    Serial.println("Failed to connect or hit timeout");
    // ESP.restart();
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println("connected...yeey :)");
    wifiok = true;
  }
  
  // Setup MQTT subscription for onoff & slider feed.
  mqtt.subscribe(&onoffbutton);
  mqtt.subscribe(&slider);
}

void checkButton(){
  // check for button press
  if ( digitalRead(TRIGGER_PIN) == LOW ) {     //Check lai nut bam sau
    // poor mans debounce/press-hold, code not ideal for production
    delay(50);
    if( digitalRead(TRIGGER_PIN) == LOW ){
      Serial.println("Button Pressed");
      // still holding button for 3000 ms, reset settings, code not ideaa for production
      delay(3000); // reset delay hold
      if( digitalRead(TRIGGER_PIN) == LOW ){
        Serial.println("Button Held");
        Serial.println("Erasing Config, restarting");
        wm.resetSettings();
        ESP.restart();
      }
      
      // start portal w delay
      Serial.println("Starting config portal");
      wm.setConfigPortalTimeout(120);
      
      if (!wm.startConfigPortal(WIFI_SSID02,WIFI_PW)) {
        Serial.println("failed to connect or hit timeout");
        delay(3000);
        // ESP.restart();
        wifiok = false;
      } 
      else {
        //if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");
        wifiok = true; 
      }
    }
  }
}


String getParam(String name){
  //read parameter from server, for customhmtl input
  String value;
  if(wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveParamCallback(){
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM customfieldid = " + getParam("customfieldid"));
}

void loop() {
  if(wm_nonblocking) wm.process(); // avoid delays() in loop when non-blocking and other long running code  
  checkButton();
  // put your main code here, to run repeatedly:
  
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  
  if (wifiok) {                                     //Neu wifi ok thi ket noi voi MQTT
  digitalWrite(S_LED, HIGH);
  MQTT_connect();
  //Sub
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    // Check if its the onoff button feed
    if (subscription == &onoffbutton) {
      Serial.print(F("On-Off button: "));
      Serial.println((char *)onoffbutton.lastread);
      
      if (strcmp((char *)onoffbutton.lastread, "ON") == 0) {
        digitalWrite(Relay1, HIGH); 
        relay_status.publish(1);
      }
      if (strcmp((char *)onoffbutton.lastread, "OFF") == 0) {
        digitalWrite(Relay1, LOW);
        relay_status.publish(0); 
      }
    }
    
    //check if its the slider feed
    if (subscription == &slider) {
      Serial.print(F("Slider: "));
      Serial.println((char *)slider.lastread);
      sliderval = atoi((char *)slider.lastread);  // convert to a number
      
      if (sliderval == 0){
         auto_mode = false;
         Serial.println("Manual mode!");
      } 
      else {
         auto_mode = true;
         Serial.println("Automatic mode !");
         Serial.print("Soil moisture threshold is:");
         Serial.println(sliderval);
        }
      }
     }
  
   if (millis() - lastPub > MQTT_UPDATE_INTERVAL) {
   /************chuong trinh chinh cho vao day***************/

  PubSoil01();
  lastPub = millis();
  }

  //Dieu khien Relay
  if (auto_mode && soil_moisture1>0) {
    ControlRelay();
  }
  // ping the server to keep the mqtt connection alive
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  }
  else {
    digitalWrite(S_LED, LOW);
    //Serial.println("Wifi Error!!!");
  }
  
}
/******************Ket thuc chuong trinh chinh********************/

//Chuong trinh con


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 5; // Thu lai 5 lan neu khong ket noi thanh cong
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 60 seconds...");
       mqtt.disconnect();
       delay(60000);  // wait 10 seconds
       retries--;
       if (retries == 0) {
        Serial.println("MQTT connection loss.");
        Serial.println("Restart ESP!");
        delay(2000);
        ESP.restart(); // Sau 5 lan khong ket noi thanh cong thi restart
         //while (1); //Dung cau lenh nay khi khong muon restart
       }
  }
  Serial.println("MQTT Connected!");
  PubSoil01();
  //MQTT_status = true;
}

//Dieu khien Relay
void ControlRelay() {
    if(soil_moisture1>sliderval){
        Serial.println("It's not enought soil moisture!");
        digitalWrite(Relay1, HIGH); 
        Serial.println("Turn on Relay!");
        relay_status.publish(1);
        delay(500);
    }
    else{
        Serial.println("Soil moisture ok!");
        digitalWrite(Relay1, LOW); 
        Serial.println("Turn off Relay!");
        relay_status.publish(0);
        delay(500);
    }
  }

 void PubSoil01(){
    soil_moisture1 = analogRead(SoilPin);
  //Publish Sensor 1
  Serial.print(F("\nSending Soil Moisture 1 "));//check
  Serial.print(soil_moisture1);
  Serial.print("...");
  if (! soil.publish(soil_moisture1)) {
    Serial.println(F("Soil1 failed to publish!"));
  } else {
    Serial.println(F("Soil1 published!"));
  }
 }
