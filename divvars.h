const char* g_ssid = "xxxxx"; //ssid på WiFi netværk
const char* g_password = "xxxxx"; //Kodeord til WiFi

bool useWebPage = false; //false hvis data ikke skal sendes til hjemmeside der lager data
const char* webserveraddr = "192.168.0.100"; //IP på lokal web server eller ekstern, kan også være f.eks. www.ditdomæne.dk
const String webserverIndexphpPath = "/path/path/";

bool useMqttDiscovery = true;
const char* g_mqtt_server = "192.168.0.44";  // 201 MQTT Server IP, same of Home Assistant
const char* g_mqttUser = "mqtt";              // MQTT Server User Name
const char* g_mqttPsw = "mqtt";               // MQTT Server password
int g_mqttPort = 1883;                        // MQTT Server Port
