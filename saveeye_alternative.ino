/*
 * Saveeye_alternative
 * Home Assistant MQTT Discovery Integration
 * Created by Harald Kroning <post@kroning.dk>
 * ----------------------------------------------------------------------------
 * REVISION HISTORY
 * Version 1.0 - maj 28, 2023
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "divvars.h"

// Initialize the Ethernet server library
WiFiServer server(80);
WiFiClient client;
WiFiClient mqttethclient;

PubSubClient mqttclient(mqttethclient);

//Saveeye pins
const int phototransistorPin = 33;
const int redledPin = 25;
const int greenledPin = 26;
const int blueledPin = 27;
const int buttonPin = 35;

const int dv = 50;  //pause i loop, giver et lidt lavere mw forbrug

const unsigned long oneMinuteMillis = 60UL * 1UL * 1000UL;  //sekunder * minutter * 1000
unsigned long startPulseRegTime = 0;
unsigned long lastMqttReconectTime = 0;
unsigned long lastWiFiCheckTime = 0;
unsigned long firstStatusSetTime = 0;

volatile unsigned long pulseLengthTime = 0;
volatile unsigned long prevPulseTime = 0;
volatile unsigned long lastirqCall = 0;
volatile int lastPinStatus = LOW;

volatile unsigned long totalBlink = 0;  //total antal blink siden opstart
volatile unsigned long dailyBlink = 0;  //total antal blink i aktuelle døgn, nulstilles kl. 00
unsigned long lastTotalBlink = 0;

portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;

const char* NTP_SERVER1 = "pool.ntp.org";
const char* NTP_SERVER2 = "dk.pool.ntp.org";
const char* TZ_INFO = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00";  // enter your time zone (https://remotemonitoringsystems.ca/time-zone-abbreviations.php)
tm timeinfo;
time_t now;
int prevtimehour = -1;
bool timeIsSet = false;
String startUpTime = "n/a";

// Variable used for MQTT Discovery
const char* g_deviceModel = "ArduinoDevice";                          // Hardware Model
const char* g_swVersion = "1.0";                                      // Firmware Version
const char* g_manufacturer = "HK";                                    // Manufacturer Name
String g_deviceName = "ElmolerSensor";                            // Device Name
String g_mqttStatusTopic = "arduinoiotsensor/" + g_deviceName;        // MQTT Topic
String g_mqttSetTopic = "arduinoiotsensor/" + g_deviceName + "/set";  // MQTT Topic
String g_UniqueId;

const char* haBirthMessageTopic = "homeassistant/status";

struct tblinkData {
  int pulseCount = 0;
  double avgWatt = 0.0;  //VIGTIG: der bruges double i stedet for float da ESP32 ikke understøtter brug af float i en interrupt
  double minWatt = 10000.0;
  double maxWatt = 0.0;
};

volatile struct tblinkData blinkData;
struct tblinkData blinkDataSend;

bool debugMode = false;

void Serialprint(String text, bool doln = true) {
  if (debugMode) {
    if (doln)
      Serial.println(text);
    else
      Serial.print(text);
  }
}

// Mqtt Callback function
void callback(char* topic, byte* payload, unsigned int length) {

  String myString = "";

  Serialprint("Message arrived [", false);
  Serialprint(topic, false);
  Serialprint("] ", false);
  for (int i = 0; i < length; i++) {
    Serialprint((String)(char)payload[i], false);
    myString += (char)payload[i];
  }
  //Serialprint('*' + myString + '*');
  Serialprint("");

  if (myString == "stopmqtt") {
    useMqttDiscovery = false;
    return;
  }
  if (myString == "startmqtt") {
    useMqttDiscovery = true;
    MqttHomeAssistantDiscovery();
    return;
  }

  if (String(topic) == String(haBirthMessageTopic)) {
    if (myString == "online")  //genstart af Home Assistant, gensend discovery
      MqttHomeAssistantDiscovery();
  }
}  // mqtt void callback

void mqttreconnect() {
  Serialprint("Connect to MQTT");

  Serialprint("Attempting MQTT connection...", false);
  // Attempt to connect
  if (mqttclient.connect(g_deviceName.c_str(), g_mqttUser, g_mqttPsw)) {
    Serialprint("connected");

    mqttclient.subscribe(haBirthMessageTopic);
    mqttclient.subscribe(g_mqttSetTopic.c_str());
  } else {
    Serialprint("failed, rc=", false);
    Serialprint((String)mqttclient.state(), false);
    Serialprint(" try again in 10 seconds");
  }
  lastMqttReconectTime = millis();
}

void setup_wifi() {
  delay(1000);
  byte mac[6];
  Serialprint("Setup WiFi");
  // start the Ethernet connection and the server:
  WiFi.begin(g_ssid, g_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serialprint(".");
  }

  //WiFi.setAutoReconnect(true);
  //WiFi.persistent(true);

  // start the server
  server.begin();
  Serialprint("WiFi connected");
  Serialprint("IP address: ");
  Serialprint(WiFi.localIP().toString());

  WiFi.macAddress(mac);
  g_UniqueId = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
  Serialprint(g_UniqueId);
}

void IRAM_ATTR blink() {
  portENTER_CRITICAL(&synch);

  int pinStatus = digitalRead(phototransistorPin);

  //der er lavet en del test her for at undgå bounce og fjerne støj impulser, kan evt. også gøres med hardware, ved ikke om saveeye har gjort noget med hardware
  if ((pinStatus == HIGH) and (lastPinStatus == LOW)) {
    lastPinStatus = HIGH;
    lastirqCall = millis();
    digitalWrite(blueledPin, LOW);
  } else if ((pinStatus == LOW) and (lastPinStatus == HIGH)) {
    unsigned long blinktime = millis() - lastirqCall;
    lastPinStatus = LOW;
    digitalWrite(blueledPin, HIGH);  //LOW for at være tændt

    if (((millis() - prevPulseTime) > 100) and (blinktime > 1)) {
      blinkData.pulseCount++;
      totalBlink++;
      dailyBlink++;
      pulseLengthTime = millis() - prevPulseTime;
      prevPulseTime = millis();

      double pwr = 0.0;

      if (pulseLengthTime > 0)
        pwr = (1000. * (3600. / (pulseLengthTime)));  // . efter int tal for at tvinge til float;
      blinkData.avgWatt = blinkData.avgWatt + pwr;
      if (pwr < blinkData.minWatt) blinkData.minWatt = pwr;
      if (pwr > blinkData.maxWatt) blinkData.maxWatt = pwr;

      if (debugMode) {
        //HUSK: ESP32 har det ikke godt med Serial print i en interrupt, så dette kan give en fejl og reboot hvis debugMode=true
        Serialprint("Pulse length: ", false);
        Serialprint(String(pulseLengthTime));
        Serialprint("Avg Watt: ", false);
        Serialprint(String(blinkData.avgWatt));
        Serialprint("Min Watt: ", false);
        Serialprint(String(blinkData.minWatt));
        Serialprint("Max Watt: ", false);
        Serialprint(String(blinkData.maxWatt));
      }
    }
  }
  portEXIT_CRITICAL(&synch);
}

void setup() {
  Serial.begin(9600);
  if (debugMode) {
    //Serial.begin(9600);
    while (!Serial && millis() < 5000)
      ;
    if (!Serial) debugMode = false;
    else {
      //pinMode(LED_BUILTIN, OUTPUT);
      //digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  setup_wifi();

  pinMode(buttonPin, INPUT);

  pinMode(redledPin, OUTPUT);
  digitalWrite(redledPin, HIGH);  //HIGH for at være tændt

  pinMode(greenledPin, OUTPUT);
  digitalWrite(greenledPin, HIGH);  //HIGH for at være slukket

  pinMode(blueledPin, OUTPUT);
  digitalWrite(blueledPin, HIGH);  //HIGH for at være slukket

  pinMode(phototransistorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(phototransistorPin), blink, CHANGE);
  startPulseRegTime = millis();

  mqttclient.setServer(g_mqtt_server, g_mqttPort);
  mqttclient.setCallback(callback);
  mqttclient.setBufferSize(1024);  // Vigtig da DISCOVERY pakker fylder over 500 og standard størrelse er 256
  //setup_wifi(); flyt op over pinmode

  configTime(0, 0, NTP_SERVER1);
  // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for Timezone codes for your region
  setenv("TZ", TZ_INFO, 1);
}

void sendWebPage(WiFiClient client) {
  Serialprint("New client");
  // an http request ends with a blank line
  String readString = "";
  boolean currentLineIsBlank = true;
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      readString += c;
      //Serial.write(c);
      // if you've gotten to the end of the line (received a newline
      // character) and the line is blank, the http request has ended,
      // so you can send a reply

      if (c == '\n' && currentLineIsBlank) {

        // send a standard http response header
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");  // the connection will be closed after completion of the response
        client.println();
        client.println(pulseLengthTime);
        break;
      }

      if (c == '\n') {
        // you're starting a new line
        currentLineIsBlank = true;
      } else if (c != '\r') {
        // you've gotten a character on the current line
        currentLineIsBlank = false;
      }
    }
  }
  // give the web browser time to receive the data
  delay(1);
  // close the connection:
  client.stop();
  Serialprint(readString);
  Serialprint("client disconnected");
}

void sendPCtoDB() {
  if (!useWebPage)
    return;

  String readString = "";

  if (client.connect(webserveraddr, 80)) {
    Serialprint("connected to ", false);
    if (debugMode)
      Serial.println(client.remoteIP());
    // Make a HTTP request:
    readString = "GET " + webserverIndexphpPath + "index.php?pc=" + String(blinkDataSend.pulseCount) + "&avgw=" + String(blinkDataSend.avgWatt) + "&minw=" + String(blinkDataSend.minWatt) + "&maxw=" + String(blinkDataSend.maxWatt) + " HTTP/1.1";
    client.println(readString);
    client.println("Host: " + String(webserveraddr));
    client.println("Connection: close");
    client.println();
  } else {
    // if you didn't get a connection to the server:
    Serialprint("connection failed");
    return;
  }

  readString = "";

  //lav en timeout her
  //while (client.connected() && !client.available()) delay(1);  //waits for data

  unsigned long startTime = millis();
  while (client.connected() && !client.available() && millis() - startTime <= 5000) delay(1);
//  while (client.connected() && !client.available()) delay(1);  //waits for data

  startTime = millis();
  while ( (client.connected() || client.available()) && (millis() - startTime <= 5000) ) { //connected or data available
  //while (client.connected() || client.available()) {           //connected or data available
    char c = client.read();                                    //gets byte from ethernet buffer
    readString += c;                                           //places captured byte in readString
  }

  Serialprint(readString);
}

void publishToHA() {
  if (useMqttDiscovery) {
    if (mqttclient.connected()) {

      float pwr = 0.0;
      if (pulseLengthTime > 0)
        pwr = (1000. * (3600. / (pulseLengthTime)));  // . efter int tal for at tvinge til float;
      String watt = String(int(pwr) + 1);             //arduino runder ned ved cast så derfor +1

      float forbrug = (totalBlink / 1000.);
      float dailyforbrug = (dailyBlink / 1000.);

      JsonDocument payload;
      payload["power"] = watt;
      payload["energy"] = String(forbrug, 2);
      payload["dailyenergy"] = String(dailyforbrug, 2);
      payload["rssi"] = String(WiFi.RSSI());
      payload["startuptime"] = startUpTime;

      time(&now);
      //localtime_r(&now, &timeinfo);

      char time_output[20];
      strftime(time_output, 20, "%d-%m-%y %T", localtime(&now));
      payload["currenttime"] = String(time_output);

      String strPayload;
      serializeJson(payload, strPayload);

      if (debugMode) {
        serializeJsonPretty(payload, Serial);
        Serial.println("");
      }

      mqttclient.publish(g_mqttStatusTopic.c_str(), strPayload.c_str());
    }
  }
}

void checkIfTimeIsSet() {
  time(&now);
  localtime_r(&now, &timeinfo);

  if (timeinfo.tm_year > 100) {
    Serialprint("Time found");
    timeIsSet = true;

    char time_output[20];
    strftime(time_output, 20, "%d-%m-%y %T", localtime(&now));

    startUpTime = String(time_output);
  }
}

void runStartupInit() {
  firstStatusSetTime = 0;
  publishToHA();
}

void loop() {
  if ((WiFi.status() != WL_CONNECTED) && ((millis() - lastWiFiCheckTime) > 30000)) {
    Serialprint("Reconnecting to WiFi");
    WiFi.disconnect();
    WiFi.reconnect();
    lastWiFiCheckTime = millis();
  }

  if (!timeIsSet) checkIfTimeIsSet();

  if ((!mqttclient.connected()) and ((millis() - lastMqttReconectTime) > 10000)) {
    mqttreconnect();

    if (mqttclient.connected()) {
      delay(100);
      MqttHomeAssistantDiscovery();  // Send Discovery Data
    }
  }

  if ((mqttclient.connected()) and (firstStatusSetTime > 0) and (millis() - firstStatusSetTime > 5000)) {
    runStartupInit();
  }

  WiFiClient client = server.available();
  if (client) {
    sendWebPage(client);
  }

  if (totalBlink != lastTotalBlink) {
    lastTotalBlink = totalBlink;
    publishToHA();
  }

  if ((millis() - startPulseRegTime) >= oneMinuteMillis) {
    blinkDataSend.pulseCount = blinkData.pulseCount;
    blinkDataSend.avgWatt = blinkData.avgWatt / blinkData.pulseCount;
    blinkDataSend.maxWatt = blinkData.maxWatt;
    blinkDataSend.minWatt = blinkData.minWatt;

    blinkData.pulseCount = 0;
    blinkData.avgWatt = 0.0;
    blinkData.maxWatt = 0.0;
    blinkData.minWatt = 10000.0;

    startPulseRegTime = millis();
    Serialprint("Pulse count: ", false);
    Serialprint(String(blinkDataSend.pulseCount));

    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_hour != prevtimehour) {
      prevtimehour = timeinfo.tm_hour;
      if (timeinfo.tm_hour == 0)
        dailyBlink = 0;
    }
  }

  if (blinkDataSend.pulseCount > 0) {
    // TODO: hvis det ikke lykkes så nulstil ikke men prøv igen efter lidt tid
    sendPCtoDB();
    blinkDataSend.pulseCount = 0;
  }

  delay(dv);
  mqttclient.loop();
}

void MqttHomeAssistantDiscovery() {
  if (!useMqttDiscovery)
    return;

  Serialprint("MqttHomeAssistantDiscovery");
  String discoveryTopic;
  //String payload;
  String strPayload;

  if (mqttclient.connected()) {
    Serialprint("SEND HOME ASSISTANT DISCOVERY!!!");
    JsonDocument payload;
    JsonObject device;
    JsonArray identifiers;

    JsonDocument json_attr_tpl;
    json_attr_tpl["power"] = "{{value_json.power}}";
    json_attr_tpl["energy"] = "{{value_json.energy}}";
    json_attr_tpl["dailyenergy"] = "{{value_json.dailyenergy}}";
    json_attr_tpl["rssi"] = "{{value_json.rssi}}";
    json_attr_tpl["startuptime"] = "{{value_json.startuptime}}";
    json_attr_tpl["currenttime"] = "{{value_json.currenttime}}";
    String str_json_attr_tpl;
    serializeJson(json_attr_tpl, str_json_attr_tpl);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Power
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    discoveryTopic = "homeassistant/sensor/arduinoiotsensor/" + g_deviceName + "_power" + "/config";

    Serialprint(discoveryTopic);

    payload["name"] = g_deviceName + ".power";
    payload["uniq_id"] = g_UniqueId + "_power";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "power";
    payload["stat_cla"] = "measurement";
    payload["val_tpl"] = "{{ value_json.power | is_defined }}";

    payload["json_attr_t"] = g_mqttStatusTopic;
    payload["json_attr_tpl"] = str_json_attr_tpl;

    payload["unit_of_meas"] = "W";
    device = payload["device"].to<JsonObject>();
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device["identifiers"].to<JsonArray>();
    identifiers.add(g_UniqueId);

    if (debugMode) {
      serializeJsonPretty(payload, Serial);
      Serial.println(" ");
    }
    serializeJson(payload, strPayload);

    if (!mqttclient.publish(discoveryTopic.c_str(), strPayload.c_str()))
      Serialprint("fejl i power");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Energy
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    payload.clear();
    //device.clear();
    //identifiers.clear();
    //strPayload = "";  //.clear();

    discoveryTopic = "homeassistant/sensor/arduinoiotsensor/" + g_deviceName + "_energy" + "/config";

    payload["name"] = g_deviceName + ".energy";
    payload["uniq_id"] = g_UniqueId + "_energy";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "energy";
    payload["stat_cla"] = "total_increasing";
    payload["val_tpl"] = "{{ value_json.energy | is_defined }}";

    payload["json_attr_t"] = g_mqttStatusTopic;
    payload["json_attr_tpl"] = str_json_attr_tpl;

    payload["unit_of_meas"] = "kWh";
    device = payload["device"].to<JsonObject>();
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device["identifiers"].to<JsonArray>();
    identifiers.add(g_UniqueId);

    if (debugMode) {
      serializeJsonPretty(payload, Serial);
      Serial.println(" ");
    }
    serializeJson(payload, strPayload);

    if (!mqttclient.publish(discoveryTopic.c_str(), strPayload.c_str()))
      Serialprint("fejl i power");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // dailyEnergy
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    payload.clear();
    //device.clear();
    //identifiers.clear();
    //strPayload = "";  //.clear();

    discoveryTopic = "homeassistant/sensor/arduinoiotsensor/" + g_deviceName + "_dailyenergy" + "/config";

    payload["name"] = g_deviceName + ".dailyenergy";
    payload["uniq_id"] = g_UniqueId + "_dailyenergy";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "energy";
    payload["stat_cla"] = "total_increasing";
    payload["val_tpl"] = "{{ value_json.dailyenergy | is_defined }}";

    payload["json_attr_t"] = g_mqttStatusTopic;
    payload["json_attr_tpl"] = str_json_attr_tpl;

    payload["unit_of_meas"] = "kWh";
    device = payload["device"].to<JsonObject>();
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device["identifiers"].to<JsonArray>();
    identifiers.add(g_UniqueId);

    if (debugMode) {
      serializeJsonPretty(payload, Serial);
      Serial.println(" ");
    }
    serializeJson(payload, strPayload);

    if (!mqttclient.publish(discoveryTopic.c_str(), strPayload.c_str()))
      Serialprint("fejl i dailyEnergy");

    delay(1000);

    firstStatusSetTime = millis();
  }
}
