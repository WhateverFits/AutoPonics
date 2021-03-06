#include <ArduinoJson.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <Timezone.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266httpUpdate.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <EasyButton.h>
#include <PubSubClient.h>
#include <stdint.h>
#include <jled.h>
#include "Config.h"
#include "AutoPonics.h"
#include "Formatting.h"

#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

Adafruit_AM2320 am2320 = Adafruit_AM2320();



// MQTT method headers
void mqttPublish(bool light, bool water);
void mqttCallback(char* topic, byte* payload, unsigned int length);

// Alarm header
void setupAlarms(bool checkState);

// WiFi connection
ESP8266WiFiMulti wifiMulti;
WiFiClient mqttWiFiClient;
PubSubClient mqttClient(MQTT_SERVER, MQTT_PORT, mqttCallback, mqttWiFiClient);

// NTP
WiFiUDP Udp;

// Control button
EasyButton button(BUTTON_PIN);

// TimeZone rules
TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};    //Daylight time = UTC - 7 hours
TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};     //Standard time = UTC - 8 hours
Timezone myTZ(myDST, mySTD);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev

// Locals
unsigned long lastTimeClock = 0;
time_t local;
byte waterduration;
float temp;
float humid;
bool lightState = false;
bool waterState = false;
bool connectedOnce = false;
String mqttClientId; 
unsigned long lastReconnectAttempt = 0; 
int ntpRetryCount = 3;
int ntpRetry = 0;

// Alarms
int lightOnIndex = -1;
int lightOffIndex = -1;
int waterOnIndex = -1;
int waterOffIndex = -1;

void LightOnAlarm() {
  Serial.println("Good morning!");
  time_t utc = now();
  time_t local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  lightState = true;
  digitalWrite(LIGHTPIN, HIGH);
  mqttPublish(lightState, waterState);
}

void LightOffAlarm() {
  Serial.println("Good evening!");
  time_t utc = now();
  time_t local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  lightState = false;
  digitalWrite(LIGHTPIN, LOW);
  mqttPublish(lightState, waterState);
}

void WaterOnAlarm() {
  Serial.println("Flood cycle started");
  time_t utc = now();
  time_t local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  waterState = true;
  digitalWrite(WATERPIN, HIGH);
  mqttPublish(lightState, waterState);
}

void WaterOffAlarm() {
  Serial.println("Flood cycle finished");
  time_t utc = now();
  time_t local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  waterState = false;
  digitalWrite(WATERPIN, LOW);
  mqttPublish(lightState, waterState);
}

int createAlarmUTC(int h, int m, OnTick_t onTickHandler) {
  TimeElements t;
  t.Second = 0;
  t.Minute = m;
  t.Hour = h;

  // Unused
  t.Day = 18;
  t.Month = 11;
  t.Year = year(now()) - 1970;

  time_t utc = makeTime(t);
  mqttLog(("UTC Alarm set for " + String(h) + ":" + String(m)));
  return Alarm.alarmRepeat(hour(utc), m, 0, onTickHandler);
}

int createAlarm(int h, int m, OnTick_t onTickHandler, String name) {
  TimeElements t;
  t.Second = 0;
  t.Minute = m;
  t.Hour = h;
  time_t holding = now();
  t.Day = day(holding);
  t.Month = month(holding);
  t.Year = year(holding) - 1970;

  time_t localTime = makeTime(t);
  time_t utc = myTZ.toUTC(localTime);
  Serial.print("createAlarm: ");
  Serial.print(hour(utc));
  Serial.print(":");
  Serial.println(m);
  mqttLog("CreateAlarm " + name + ":"` + String(hour(utc)) + ":" + Strimg(m));
  return Alarm.alarmRepeat(hour(utc), m, 0, onTickHandler);
}

void setupAlarms(bool checkState) {
  mqttLog("setupAlarms - Enter");
  for (int i = 0; i < dtNBR_ALARMS; i++) {
    Alarm.free(i);
  }

  byte hour;
  byte minute;
  byte lightduration;
  byte waterduration;
  EEPROM.get(FIXEDHOURINDEX, hour);
  EEPROM.get(FIXEDMINUTEINDEX, minute);
  EEPROM.get(WATERDURATIONINDEX, waterduration);
  EEPROM.get(LIGHTDURATIONINDEX, lightduration);

  waterOnIndex = createAlarm(hour, minute, WaterOnAlarm, "WaterOn");
  byte waterOnH = hour;
  byte waterOnM = minute;

  minute += waterduration;
  normalizeTime(&hour, &minute);
  waterOffIndex = createAlarm(hour, minute, WaterOffAlarm, "WaterOff");
  byte waterOffH = hour;
  byte waterOffM = minute;

  minute += 5;
  normalizeTime(&hour, &minute);
  lightOnIndex = createAlarm(hour, minute, LightOnAlarm, "LightOn");
  byte lightOnH = hour;
  byte lightOnM = minute;

  hour += lightduration;
  normalizeTime(&hour, &minute);
  lightOffIndex = createAlarm(hour, minute, LightOffAlarm, "LightOff");

  if (checkState) {
    Serial.println("Checking state");
    time_t utc = now();
    time_t local = myTZ.toLocal(utc, &tcr);

    TimeElements t;
    t.Second = 0;
    t.Minute = lightOnM;
    t.Hour = lightOnH;
    time_t holding = now();
    t.Day = day(holding);
    t.Month = month(holding);
    t.Year = year(holding) - 1970;

    time_t onTime = makeTime(t);

    t.Minute = minute;
    t.Hour = hour;
    time_t offTime = makeTime(t);

    if (local >= onTime && local <= offTime){
      Serial.println("Turning on because it should have been on.");
      LightOnAlarm();
    } else {
      LightOffAlarm();
    }
  }

  mqttLog("setupAlarms - Exit");
}

void normalizeTime(byte *hour, byte *minute) {
  if (*minute > 60) {
    *minute -= 60;
    *hour++;
  }

  if (*hour == 24) {
    *hour = 0;
  }
}
// When the button is short pressed, execute this
void onPressed() {
  if (lightState)
    LightOffAlarm();
  else
    LightOnAlarm();
  lastTimeClock = millis();
}

// When the button is held for 1000 ms, execute this
void onPressedForDuration() {
  Serial.println("Fast Toggle from long press");

  // Reset the clock timer so that the RISE or SET displays for a second
  if (waterState)
    WaterOffAlarm();
  else
  {
    WaterOnAlarm();
    byte h = hour(now());
    byte m = minute(now()) + waterduration;
    normalizeTime(&h, &m);
    createAlarm(h, m, WaterOffAlarm, "WaterOff");
  }
  lastTimeClock = millis();
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Udp.beginMulticast(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
    String(ipAddress[1]) + String(".") +\
    String(ipAddress[2]) + String(".") +\
    String(ipAddress[3])  ;
}

time_t getNtpTime()
{
  if (wifiMulti.run() == WL_CONNECTED) {
    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    // Fall back to using the global NTP pool server in case we cannot connect to internal NTP server
    if (ntpRetry > 1)
      WiFi.hostByName(ntpServerName, timeServer);
    Serial.print("Transmit NTP Request to ");
    Serial.println(IpAddress2String(timeServer));
    sendNTPpacket(timeServer);
    uint32_t beginWait = millis();
    byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
    while (millis() - beginWait < 1500) {
      int size = Udp.parsePacket();
      if (size >= NTP_PACKET_SIZE) {
        Serial.println("Receive NTP Response");
        Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
        unsigned long secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
        secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
        secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
        secsSince1900 |= (unsigned long)packetBuffer[43];
        randomSeed(secsSince1900);
        // Make sure we chill a little
        setSyncInterval(200);
        unsigned long calcTime = secsSince1900 - 2208988800UL;// + timeZone * SECS_PER_HOUR;
        Serial.print("ntp: ");
        Serial.println(calcTime);
        return calcTime;
      }
    }
  }
  Serial.println("No NTP Response :-(");
  if (ntpRetry < ntpRetryCount) 
  {
    ntpRetry++;
    return getNtpTime();
  }

  // We couldn't connect so we are gonna try harder!
  Serial.println("NTP n");
  setSyncInterval(5);
  return now(); // return now if unable to get the time so we just get our drifted internal time instead of wrong time.
}

// The callback when an MQTT message is received. We only listen to one topic (control)
// so we only pay attenttion to the message/payload/action.
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Inside mqtt callback: %s\n", topic);
  Serial.println(length);

  // This payload can be the entire buffer. Make sure to only use the provided length of it.
  String action = (char*)payload;
  action = action.substring(0, length);
  Serial.println(action);

  //if (action == "Sunrise") sunrise.StartSunrise();
  if (action == "WaterOn") {
    WaterOnAlarm();
  }
  else if (action == "WaterOff") {
    WaterOffAlarm();
  }
  else if (action == "LightOn") {
    LightOnAlarm();
  }
  else if (action == "LightOff") {
    LightOffAlarm();
  }
  if (action.indexOf("SetStartTime=")== 0) {
    String time = action.substring(13);
    int idx = action.indexOf(':');
    String hourStr = action.substring(0,idx);
    String minStr = action.substring(idx+1);
    int hour = hourStr.toInt();
    int minute = minStr.toInt();
    if (hour < 24 && hour >= 0 && minute < 60 && minute >= 0) {
      EEPROM.put(FIXEDHOURINDEX, (byte)hour);
      EEPROM.put(FIXEDMINUTEINDEX, (byte)minute);
      EEPROM.commit();
    }
    setupAlarms(true);
  }
  if (action.indexOf("SetWaterMin=")== 0) {
    String time = action.substring(12);
    int minutes = time.toInt();
    if (minutes < 60 && minutes > 0) {
      EEPROM.put(WATERDURATIONINDEX, (byte)minutes);
      EEPROM.commit();
    }
    setupAlarms(true);
  }
  if (action.indexOf("SetLightHour=")== 0) {
    String time = action.substring(13);
    int hours = time.toInt();
    if (hours < 20 && hours > 0) {
      EEPROM.put(LIGHTDURATIONINDEX, (byte)hours);
      EEPROM.commit();
    }
    setupAlarms(true);
  }

  if (action == "Update") {
    WiFiClient updateWiFiClient;
    t_httpUpdate_return ret = ESPhttpUpdate.update(updateWiFiClient, UPDATE_URL);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}

// Send out the provided message in JSON format to home/DNSNAME/state
void mqttPublish(bool light, bool water) {
  if (mqttClient.connected()) {
    DynamicJsonDocument mqttDoc(1024);
    mqttDoc["LightStatus"] = light ? "On" : "Off";
    mqttDoc["WaterStatus"] = water ? "On" : "Off";
    time_t utc = now();
    time_t local = myTZ.toLocal(utc, &tcr);
    mqttDoc["Date"] = formatTime(local, tcr -> abbrev);
    char buffer[512];
    size_t n = serializeJson(mqttDoc, buffer);
    mqttClient.publish(MQTT_CHANNEL_PUB, buffer, true);
  }
}

void mqttLog(const char* status) {
  Serial.println(status);
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_CHANNEL_LOG, status, true);
  }
}

void mqttLog(String status) {
  char buf[256];
  status.toCharArray(buf, 256);
  mqttLog(buf);
}

boolean mqttReconnect() {
  char buf[100];
  mqttClientId.toCharArray(buf, 100);
  if (mqttClient.connect(buf, MQTT_USER, MQTT_PASSWORD)) {
    mqttClient.subscribe(MQTT_CHANNEL_SUB);
    Serial.println("MQTT connected");
  }

  return mqttClient.connected();
}

String generateMqttClientId() {
  char buffer[4];
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  sprintf(buffer, "%02x%02x", macAddr[4], macAddr[5]);
#ifdef DNSNAME
  return DNSNAME + String(buffer);
#else
  return "AutoPonics" + String(buffer);
#endif
}

void updateStarted() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void updateFinished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void updateProgress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void updateError(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch(event) {
    case WIFI_EVENT_STAMODE_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      onConnect();
      break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
      Serial.println("WiFi lost connection");
      WiFi.disconnect();
      connectedOnce = false;
      delay(500);
      wifiMulti.run();
      break;
  }
}

void onConnect() {
  Serial.print("Connected: ");
  Serial.println(WiFi.localIP());
  connectedOnce = true;
  Udp.begin(LOCALUDPPORT);
  setSyncProvider(getNtpTime);
  mqttPublish(lightState, waterState);
}

void setupWiFi(){
  WiFi.disconnect();
  delay(1000);
  //WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);
#ifdef DNSNAME
  WiFi.hostname(DNSNAME);
#else
  char buffer[4];
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  sprintf(buffer, "%02x%02x", macAddr[4], macAddr[5]);
  WiFi.hostname("AutoPonics" + String(buffer));
#endif

  for (int i=0; i < wifiCount; i++) {
    wifiMulti.addAP(ssids[i], passs[i]);
    Serial.println(ssids[i]);
  }
  Serial.println("Connecting");
  wifiMulti.run();
  //WiFi.begin(ssids[0], passs[0]);
}

bool validateWiFi(unsigned long milliseconds) {
  // Update WiFi status. Take care of rollover
  if (milliseconds >= lastTimeClock + 1000 || milliseconds < lastTimeClock) {
    lastTimeClock = milliseconds;
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("Disconnected");
      connectedOnce = false;
    } else {
      if (!connectedOnce) {
        Serial.print("Connected late to ");
        Serial.println(WiFi.SSID());
        onConnect();
      }

      connectedOnce = true;
    }
  }

  return connectedOnce;
}

void validateMqtt(unsigned long milliseconds) {
  if (!mqttClient.connected()) {
    if (milliseconds - lastReconnectAttempt > 5000 || milliseconds < lastReconnectAttempt) {
      lastReconnectAttempt = milliseconds;
      Serial.println("MQTT not connected");
      // Attempt to reconnect
      if (mqttReconnect()) {
        Serial.println("MQTT reconnected");
      }
    }

    if (milliseconds - lastReconnectAttempt > 60000) {
      Serial.println("MQTT disconnecting WiFi");
      WiFi.disconnect();
      delay(500);
    }
  } else {
    mqttClient.loop();
  }
}

void mqttPublishTempHumid(float temperature, float humidity) {
  char buf[256];
  if (mqttClient.connected()) {
    String(temperature).toCharArray(buf, 256);
    mqttClient.publish(MQTT_CHANNEL_TEMP, buf, true);
    String(humidity).toCharArray(buf, 256);
    mqttClient.publish(MQTT_CHANNEL_HUMID, buf, true);
  }
}

long lastTimeSense = 0;
// Get the temperature and humidity
bool getTempHumid(long milliseconds) {
  if (milliseconds >= lastTimeSense + 600000 || lastTimeSense == 0 || milliseconds < lastTimeSense) {
    lastTimeSense = milliseconds;
    temp = am2320.readTemperature();
    humid = am2320.readHumidity();
    if (isnan(humid) || isnan(temp)) {
      Serial.println("Failed to read from DHT sensor!");
      return false;
    }
    Serial.print("Temp: "); Serial.println(temp);
    Serial.print("Hum: "); Serial.println(humid);
    return true;
  }

  return false;
}

// Publish the temperature and humidity for monitoring
void logTempHumid() {
  if (mqttReconnect()) {
    mqttPublishTempHumid(temp, humid);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println();

  pinMode(LIGHTPIN, OUTPUT);
  pinMode(WATERPIN, OUTPUT);

  setupWiFi();

  EEPROM.begin(10);

  button.begin();
  // Add the callback function to be called when the button is pressed.
  button.onPressed(onPressed);
  button.onPressedFor(1000, onPressedForDuration);

  mqttClientId = generateMqttClientId();

  ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
  ESPhttpUpdate.onStart(updateStarted);
  ESPhttpUpdate.onEnd(updateFinished);
  ESPhttpUpdate.onProgress(updateProgress);
  ESPhttpUpdate.onError(updateError);
  setupAlarms(false);
  Serial.print("Current time:"); Serial.println(now());
}

bool checkedStatus = false;
void loop() {
  Alarm.delay(0);
  button.read();

  unsigned long milliseconds = millis();

  // Check if connected then handle the connected magic
  if (validateWiFi(milliseconds)) {
    validateMqtt(milliseconds);

    if (getTempHumid(milliseconds)) {
      logTempHumid();
    }

    if (!checkedStatus){
      setupAlarms(true);
      checkedStatus = true;
    }
  }
  delay(50);
}
