#ifndef _CONFIG
#define _CONFIG

#define FIXEDHOURINDEX 0
#define FIXEDMINUTEINDEX FIXEDHOURINDEX + sizeof(byte)
#define WATERDURATIONINDEX FIXEDMINUTEINDEX + sizeof(byte)
#define LIGHTDURATIONINDEX WATERDURATIONINDEX + sizeof(byte)
#define LIGHTPIN D1
#define WATERPIN D2
#define BUTTON_PIN D3
#define DNSNAME "AutoPorch"

#define LOCALUDPPORT 8888
#define MQTT_SERVER "pi4"
#define MQTT_PORT 1883
#define MQTT_CHANNEL_PUB "home/" DNSNAME "/state"
#define MQTT_CHANNEL_SUB "home/" DNSNAME "/control"
#define MQTT_CHANNEL_LOG "home/" DNSNAME "/log"
#define MQTT_USER "clockuser"
#define MQTT_PASSWORD "clockuser"
#define UPDATE_URL "http://pi4/cgi-bin/test.rb"

// Try our router first to see if the timeserver is up on it.
IPAddress timeServer(192, 168, 1, 1);
//Then we try the pool.
const char* ntpServerName = "us.pool.ntp.org";

const char* ssids[] = {"Wifi", "Info"};
const char* passs[] = {"Goes", "Here"};
const int wifiCount = 1;
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message

#endif
