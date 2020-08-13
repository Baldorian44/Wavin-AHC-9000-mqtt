#include <FS.h>          // this needs to be first, or it all crashes and burns...
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager (Very important when installing this version you will get an error, thus you need the development branch.. i downloaded the development branch from github and placed here: G:\Mit programmering\Arduino Programs\libraries
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "WavinController.h"
//#include "PrivateConfig.h"
#include <DNSServer.h>
#include <Ticker.h>


/*#define mqtt_server       "192.168.1.132"
#define mqtt_port         1883
#define mqtt_user         "Mosquitto"
#define mqtt_pass         "Broker"
*/

Ticker ticker;

// MQTT defines
// Esp8266 MAC will be added to the device name, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/3/target', where 3 is the output id and XXXXXXXXXXXX is the mac
const String   MQTT_PREFIX              = "heat/";       // include tailing '/' in prefix
const String   MQTT_DEVICE_NAME         = "floor";       // only alfanumeric and no '/'
const String   MQTT_ONLINE              = "/online";      
const String   MQTT_SUFFIX_CURRENT      = "/current";    // include heading '/' in all suffixes
const String   MQTT_SUFFIX_SETPOINT_GET = "/target";
const String   MQTT_SUFFIX_SETPOINT_SET = "/target_set";
const String   MQTT_SUFFIX_MODE_GET     = "/mode";
const String   MQTT_SUFFIX_MODE_SET     = "/mode_set";
const String   MQTT_SUFFIX_BATTERY      = "/battery";
const String   MQTT_SUFFIX_OUTPUT       = "/output";

const String   MQTT_VALUE_MODE_STANDBY  = "off";
const String   MQTT_VALUE_MODE_MANUAL   = "heat";

const String   MQTT_CLIENT = "Wavin-AHC-9000-mqtt";      // mqtt client_id prefix. Will be suffixed with Esp8266 mac to make it unique

#define CUSTOM_HOSTNAME "Wavin-Gateway"                  // Hostname of the ESP8266 (Wifi + OTA) so that it's easier to find in your DHCP range

String mqttDeviceNameWithMac;
String mqttClientWithMac;

// Operating mode is controlled by the MQTT_SUFFIX_MODE_ topic.
// When mode is set to MQTT_VALUE_MODE_MANUAL, temperature is set to the value of MQTT_SUFFIX_SETPOINT_
// When mode is set to MQTT_VALUE_MODE_STANDBY, the following temperature will be used
const float STANDBY_TEMPERATURE_DEG = 5.0;

const uint8_t TX_ENABLE_PIN = 5;
const bool SWAP_SERIAL_PINS = true;
const uint16_t RECIEVE_TIMEOUT_MS = 1000;
WavinController wavinController(TX_ENABLE_PIN, SWAP_SERIAL_PINS, RECIEVE_TIMEOUT_MS);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastUpdateTime = 0;

const uint16_t POLL_TIME_MS = 5000;

const uint16_t DELAY_BETWEEN_TRANSMISSION = 1000; // Micro Seconds delayMicroseconds(DELAY_BETWEEN_TRANSMISSION);

// Web Server on port 80
ESP8266WebServer server(80);
long int AmountTempRead = 0;
long int AmountTempSetRead = 0;
long int AmountDataSendFromHA = 0;
long int AmountCurTempRead = 0;
long int AmountModeRead = 0;
long int AmountStatusRead = 0;
long int TimesRunLoop = 0;
long int TimesWifiConnected = 0;
long int TimesMQTTConnected = 0;

//define your default values here, if there are different values in config.json, they are overwritten.
/*
char mqtt_server[40] = "192.168.1.132";
char mqtt_port[6]  = "1883";
char mqtt_user[40] = "Mosquitto";
char mqtt_pass[40] = "Broker";
*/

char mqtt_server[40];
char mqtt_port[6];
char mqtt_user[40];
char mqtt_pass[40];

//default custom static IP
char static_ip[16] = "10.0.1.56";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";

//flag for saving data
bool shouldSaveConfig = false;

void tick()
{
  //toggle state
  digitalWrite(26, !digitalRead(26));     // set pin to the opposite state
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  //Serial.println("Should save config");
  shouldSaveConfig = true;
  ticker.attach(0.6, tick);
}

struct lastKnownValue_t {
  uint16_t temperature;
  uint16_t setpoint;
  uint16_t battery;
  uint16_t status;
  uint16_t mode;
} lastSentValues[WavinController::NUMBER_OF_CHANNELS];

const uint16_t LAST_VALUE_UNKNOWN = 0xFFFF;

bool configurationPublished[WavinController::NUMBER_OF_CHANNELS];


// Read a float value from a non zero terminated array of bytes and
// return 10 times the value as an integer
uint16_t temperatureFromString(String payload)
{
  float targetf = payload.toFloat();
  return (unsigned short)(targetf * 10);
}


// Returns temperature in degrees with one decimal
String temperatureAsFloatString(uint16_t temperature)
{
  float temperatureAsFloat = ((float)temperature) / 10;
  return String(temperatureAsFloat, 1);
}


uint8_t getIdFromTopic(char* topic)
{
  unsigned int startIndex = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/").length();
  int i = 0;
  uint8_t result = 0;

  while(topic[startIndex+i] != '/' && i<3)
  {
    result = result * 10 + (topic[startIndex+i]-'0');
    i++;
  }

  return result;
}


void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  String topicString = String(topic);
  
  char terminatedPayload[length+1];
  for(unsigned int i=0; i<length; i++)
  {
    terminatedPayload[i] = payload[i];
  }
  terminatedPayload[length] = 0;
  String payloadString = String(terminatedPayload);

  uint8_t id = getIdFromTopic(topic);

  if(topicString.endsWith(MQTT_SUFFIX_SETPOINT_SET))
  {
    uint16_t target = temperatureFromString(payloadString);
    wavinController.writeRegister(WavinController::CATEGORY_PACKED_DATA, id, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, target);
  }
  else if(topicString.endsWith(MQTT_SUFFIX_MODE_SET))
  {
    if(payloadString == MQTT_VALUE_MODE_MANUAL) 
    {
      wavinController.writeMaskedRegister(
        WavinController::CATEGORY_PACKED_DATA,
        id,
        WavinController::PACKED_DATA_CONFIGURATION,
        WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL,
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
    else if (payloadString == MQTT_VALUE_MODE_STANDBY)
    {
      wavinController.writeMaskedRegister(
        WavinController::CATEGORY_PACKED_DATA, 
        id, 
        WavinController::PACKED_DATA_CONFIGURATION, 
        WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY, 
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
  }

  // Force re-read of registers from controller now
  lastUpdateTime = 0;

}


void resetLastSentValues()
{
  for(int8_t i=0; i<WavinController::NUMBER_OF_CHANNELS; i++)
  {
    lastSentValues[i].temperature = LAST_VALUE_UNKNOWN;
    lastSentValues[i].setpoint = LAST_VALUE_UNKNOWN;
    lastSentValues[i].battery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].status = LAST_VALUE_UNKNOWN;
    lastSentValues[i].mode = LAST_VALUE_UNKNOWN;

    configurationPublished[i] = false;
  }
}


void publishIfNewValue(String topic, String payload, uint16_t newValue, uint16_t *lastSentValue)
{
  if (newValue != *lastSentValue)
  {
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true))
    {
        *lastSentValue = newValue;
    }
    else
    {
      *lastSentValue = LAST_VALUE_UNKNOWN;
    }
  }
}


// Publish discovery messages for HomeAssistant
// See https://www.home-assistant.io/docs/mqtt/discovery/
void publishConfiguration(uint8_t channel)
{
  String climateTopic = String("homeassistant/climate/" + mqttDeviceNameWithMac + "/" + channel + "/config");
  String climateMessage = String(
    "{\"name\": \"" +mqttDeviceNameWithMac + "_" + channel +  "_climate\", "
    "\"current_temperature_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_CURRENT + "\", " 
    "\"temperature_command_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_SETPOINT_SET + "\", " 
    "\"temperature_state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_SETPOINT_GET + "\", " 
    "\"mode_command_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_MODE_SET + "\", " 
    "\"mode_state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_MODE_GET + "\", " 
    "\"modes\": [\"" + MQTT_VALUE_MODE_MANUAL + "\", \"" + MQTT_VALUE_MODE_STANDBY + "\"], " 
    "\"availability_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE +"\", "
    "\"payload_available\": \"True\", "
    "\"payload_not_available\": \"False\", "
    "\"qos\": \"0\"}"
  );
  
  String batteryTopic = String("homeassistant/sensor/" + mqttDeviceNameWithMac + "/" + channel + "/config");
  String batteryMessage = String(
    "{\"name\": \"" +mqttDeviceNameWithMac + "_" + channel +  "_battery\", "
    "\"state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/battery\", " 
    "\"availability_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE +"\", "
    "\"payload_available\": \"True\", "
    "\"payload_not_available\": \"False\", "
    "\"device_class\": \"battery\", "
    "\"unit_of_measurement\": \"%\", "
    "\"qos\": \"0\"}"
  );

  mqttClient.publish(climateTopic.c_str(), climateMessage.c_str(), true);  
  mqttClient.publish(batteryTopic.c_str(), batteryMessage.c_str(), true);
  
  configurationPublished[channel] = true;
}

void handleWebServ(){
  String page = "<html lang=en-EN><head><meta http-equiv='refresh' content='20'/>";
  page += "<title>ESP8266 to Waving WebServer</title>";
  page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";
  page += "</head><body><h1>ESP8266 to Wavin WebServer</h1>";
  page += "<li>AmountSetTempRead: ";
  page += AmountTempSetRead;
  page += "</li>";
  page += "<li>AmountCurTempRead: ";
  page += AmountCurTempRead;
  page += "</li>";     
  page += "<li>AmountModeRead: ";
  page += AmountModeRead;
  page += "</li>";   
  page += "<li>AmountStatusRead: ";
  page += AmountStatusRead;
  page += "</li>";     
  page += "<li>TimesRunLoop: ";
  page += TimesRunLoop;
  page += "</li>"; 
  page += "<li>TimesWifiConnected: ";
  page += TimesWifiConnected;
  page += "</li>"; 
  page += "<li>TimesMQTTConnected: ";
  page += TimesMQTTConnected;
  page += "</li>";   
  page += "<li>mqtt_port: ";
  page += mqtt_port;
  page += "</li>";     
  page += "<li>mqtt_server: ";
  page += mqtt_server;
  page += "</li>"; 
  page += "<li>mqtt_user: ";
  page += mqtt_user;
  page += "</li>";      
  page += "</body></html>";
  //return page;
  server.send ( 200, "text/html", page );
}

void setupSpiffs()
{
  //clean FS, for testing
  // SPIFFS.format();

  //read configuration from FS json
 // Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
//    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
    //  Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
    //    Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        //json.printTo(Serial);
        serializeJson(json, Serial);
        //if (json.success()) {
        if (!deserializeError)
        {
    //      Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);

         

        } else {
     //     Serial.println("failed to load json config");
        }
      }
    }
  } else {
 //   Serial.println("failed to mount FS");
  }
  //end read
}

void CheckWifiAndMQTTSettings()
{
  setupSpiffs();

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  ticker.attach(0.2, tick);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, 40);


// Reset Wifi settings for testing  
  wifiManager.resetSettings();



  //set static ip
//  wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimum quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  //  if (!wifiManager.autoConnect("WavinMQTTAdapter", "password")) {
  if (!wifiManager.autoConnect("WavinMQTTAdapter")) {
    //Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
 // Serial.println("connected...yeey :)");
  ticker.detach(); 
  digitalWrite(26, 1);  

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
 // strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) 
  {
 //   Serial.println("saving config");
    DynamicJsonDocument json(1024);
    //JsonObject json = jsonDocument.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"]   = mqtt_port;
    json["mqtt_user"]   = mqtt_user;
    json["mqtt_pass"]   = mqtt_pass;

    json["ip"]          = WiFi.localIP().toString();
    json["gateway"]     = WiFi.gatewayIP().toString();
    json["subnet"]      = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
 //     Serial.println("failed to open config file for writing");
    }

    //json.printTo(Serial);
    serializeJson(json, Serial);
    //json.printTo(configFile);
    serializeJson(json, configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
  }


//  Serial.println("local ip");
//  Serial.println(WiFi.localIP());
//  client.setServer(mqtt_server, 12025);
//  const uint16_t mqtt_port_x = 12025; 
//  client.setServer(mqtt_server, mqtt_port_x);
//  client.setServer(mqtt_server, mqtt_port);
}




void setup()
{
  uint8_t mac[6];
  WiFi.macAddress(mac);

  char macStr[13] = {0};
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  mqttDeviceNameWithMac = String(MQTT_DEVICE_NAME + macStr);
  mqttClientWithMac = String(MQTT_CLIENT + macStr);

  CheckWifiAndMQTTSettings();

  //mqttClient.setServer(MQTT_SERVER.c_str(), MQTT_PORT);
  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  mqttClient.setCallback(mqttCallback);

  // Starting the web server
  server.on( "/", handleWebServ );
  server.begin();  

  WiFi.hostname(CUSTOM_HOSTNAME);

  // *********** REMOTE UPDATING CODE START (OTA) *************
  //ArduinoOTA.setHostname(CUSTOM_HOSTNAME);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}


void loop()
{
  if (WiFi.status() != WL_CONNECTED) 
  {
    //WiFi.mode(WIFI_STA);
    //WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());
    WiFi.begin();
    //ESP.reset();
    

    if (WiFi.waitForConnectResult() != WL_CONNECTED) return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    
    server.handleClient();
    ArduinoOTA.handle();
    if (!mqttClient.connected())
    {
      TimesMQTTConnected++;
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
      //if (mqttClient.connect(mqttClientWithMac.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str(), will.c_str(), 1, true, "False") )
      if (mqttClient.connect(mqttClientWithMac.c_str(), mqtt_user, mqtt_pass, will.c_str(), 1, true, "False") )
      {
          TimesWifiConnected++;
          String setpointSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_SETPOINT_SET);
          mqttClient.subscribe(setpointSetTopic.c_str(), 1);
          
          String modeSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_MODE_SET);
          mqttClient.subscribe(modeSetTopic.c_str(), 1);
          
          mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, true);

          // Forces resending of all parameters to server
          resetLastSentValues();
      }
      else
      {
          return;
      }
    }
  
    // Process incomming messages and maintain connection to the server
    if(!mqttClient.loop())
    {
        return;
    }

    if (lastUpdateTime + POLL_TIME_MS < millis())
    {
      lastUpdateTime = millis();

      uint16_t registers[11];
      TimesRunLoop++;
      for(uint8_t channel = 0; channel < WavinController::NUMBER_OF_CHANNELS; channel++)
      {
       /* if (channel == 5)
        {
          continue; // Channel 5 is not used (merged with kitchen for 6)
        }
        */
        if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
        {
          uint16_t primaryElement = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ELEMENT_MASK;
          bool allThermostatsLost = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ALL_TP_LOST_MASK;

          if(primaryElement==0)
          {
              // Channel not used
              continue;
          }
          
          if(!configurationPublished[channel])
          {
            uint16_t standbyTemperature = STANDBY_TEMPERATURE_DEG * 10;
            wavinController.writeRegister(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_STANDBY_TEMPERATURE, standbyTemperature);
            publishConfiguration(channel);
          }

          delayMicroseconds(DELAY_BETWEEN_TRANSMISSION);

          // Read the current setpoint programmed for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 1, registers))
          {
            uint16_t setpoint = registers[0];

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_SETPOINT_GET);
            String payload = temperatureAsFloatString(setpoint);

            //SetTempChan0_Last = SetTempChan0;
            //SetTempChan0 = ((double)setpoint/10);
            AmountTempSetRead++;

            publishIfNewValue(topic, payload, setpoint, &(lastSentValues[channel].setpoint));
            delayMicroseconds(DELAY_BETWEEN_TRANSMISSION);
          }

          // Read the current mode for the channel
          if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_CONFIGURATION, 1, registers))
          {
            uint16_t mode = registers[0] & WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK; 
            AmountModeRead++;
            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_MODE_GET);
            if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY)
            {
              publishIfNewValue(topic, MQTT_VALUE_MODE_STANDBY, mode, &(lastSentValues[channel].mode));
            }
            else if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL)
            {
              publishIfNewValue(topic, MQTT_VALUE_MODE_MANUAL, mode, &(lastSentValues[channel].mode));
            }            
            delayMicroseconds(DELAY_BETWEEN_TRANSMISSION);
          }
          
          // Read the current status of the output for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_TIMER_EVENT, 1, registers))
          {
            uint16_t status = registers[0] & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK;

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_OUTPUT);
            String payload;
            if (status & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK)
              payload = "on";
            else
              payload = "off";

            AmountStatusRead++;
            publishIfNewValue(topic, payload, status, &(lastSentValues[channel].status));
            delayMicroseconds(DELAY_BETWEEN_TRANSMISSION);
          }
          
          // If a thermostat for the channel is connected to the controller
          if(!allThermostatsLost)
          {
            // Read values from the primary thermostat connected to this channel 
            // Primary element from controller is returned as index+1, so 1 i subtracted here to read the correct element
            if (wavinController.readRegisters(WavinController::CATEGORY_ELEMENTS, primaryElement-1, 0, 11, registers))
            {
              uint16_t temperature = registers[WavinController::ELEMENTS_AIR_TEMPERATURE];
              uint16_t battery = registers[WavinController::ELEMENTS_BATTERY_STATUS]; // In 10% steps

              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_CURRENT);
              String payload = temperatureAsFloatString(temperature);

              publishIfNewValue(topic, payload, temperature, &(lastSentValues[channel].temperature));

              topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_BATTERY);
              payload = String(battery*10);

              AmountCurTempRead++;

              publishIfNewValue(topic, payload, battery, &(lastSentValues[channel].battery));
              delayMicroseconds(DELAY_BETWEEN_TRANSMISSION);
            }
          }        
        }

        // Process incomming messages and maintain connection to the server
        if(!mqttClient.loop())
        {
            return;
        }
      }
    }
  }
}
