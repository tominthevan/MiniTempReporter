#include <Arduino.h>
#include <SerialCommand.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "scheduler.h"

#define MAXSSID (16)
#define MAXWIFIPW (16)
#define MAXMQTTIP (16)
#define MAXMQTTPW (16)
#define MAXMQTTTLTOPIC (16)
#define MAXDEVICENAME (16)
#define ONEWIREPORT D2
#define MAXOWDEVICES (10)
#define MAXPORTNO (15)
 
// Create the serial command parser
SerialCommand serCmd;

// Device configuration command processing
// Device is set up for operation  by connecting
// it to a USB port and a serial monitor to enter
// commands to provide the configuration information.
// 
// Primary Function Data Structures
WiFiClient wifiClient;
PubSubClient psClient(wifiClient);
OneWire oneWire(ONEWIREPORT);
DallasTemperature ds(&oneWire);

////////////////////////////////////////////////
// Configuration data initialization and storage
////////////////////////////////////////////////

// Persistent Configuration data stored in esp8266 simulated EEPROM (spiffs)

unsigned int const configAddr = 0;  // Config data stored at offest 0
struct ConfigData{
  // WIFI Network parameters
  char wifissid[MAXSSID];           //WIFI SSID
  char wifipw[MAXWIFIPW];           //WIFI Password

  // MQTT Server parameters
  char mqttip[MAXMQTTIP];           //MQTT Server IP address
  char mqttpw[MAXMQTTPW];           //MQTT Server Password or "\0" if none
  char mqtttltopic[MAXMQTTTLTOPIC]; //MQTT top level topic for this device
  char devname[MAXDEVICENAME];      //Unique device name used for pub/sub

  //Onewire bus parameters
  DeviceAddress owDevices[MAXOWDEVICES];
  int owDevCount = 0;
  byte owPort = ONEWIREPORT;

  //Sensor reporting parameters
  //Maximum interval without report (seconds) even if unchanged
  int reportIntvl = 60;
  // report style 0 = integer e.g.  12
  //              1 = integer value in tenths of a unit e.g. 123
  //              tentths of a unit with a decimal point e.g. 12.3
  int repStyle = 0;

  bool dirty = false;                // dirty flag - during configuration becomes true
                                    // if configdata needs to be written back to EEPROM

} configData;

//void printch(String l, char *s){
//  Serial.print(l);Serial.print(s);Serial.print("[");Serial.print(strlen(s));Serial.println("]");
//}

// Process a configuration parameter - return true if config data is updated
bool processConfigParam(char *curVal, const char *newVal, unsigned int maxLength, const char *errMsg){
  
  if(newVal != NULL){                            // NULL indicates end of input line has been reached
    const char *s = newVal;
    if(strcmp(s,"\\") != 0){                    // single \ skips a parameter
      if (strcmp(s,"\"\"") == 0) s = (char *)""; // "" is translated to empty string
      if(strlen(s) < maxLength){
        if(strcmp(curVal, s) != 0){
          strcpy(curVal,s);
          return true;
        }
      } else {
        Serial.println(errMsg);
      }
    }  
  }
  return false;
}

// WIFI network and password setup
void wifiCmd(){
  
    configData.dirty |= processConfigParam(configData.wifissid, serCmd.next(), MAXMQTTPW, "MQTT IP address must be 31 or fewer characters");
    configData.dirty |= processConfigParam(configData.wifipw, serCmd.next(), MAXMQTTPW, "MQTT password must be 15 or fewer characters");
}

// MQTT IP address and password setup
// command format: mqtt ip-address password topleveltopic devicename
void mqttCmd(){
  IPAddress ipad;
  char ipstr[MAXMQTTIP];

  if(processConfigParam(ipstr, serCmd.next(), MAXSSID, "MQTT IP address must be 15 or fewer characters")){
    if(ipad.fromString(ipstr)){
      strcpy(configData.mqttip, ipstr);
      configData.dirty = true;
    } else{
      Serial.print("Invalid IP addres:");
      Serial.println(ipstr);
    }
  }
  configData.dirty |= processConfigParam(configData.mqttpw, serCmd.next(), MAXWIFIPW, "MQTT password must be 15 or fewer characters");
  configData.dirty |= processConfigParam(configData.mqtttltopic, serCmd.next(), MAXMQTTTLTOPIC, "MQTTtop level topic must be 31 or fewer characters");
  configData.dirty |= processConfigParam(configData.devname, serCmd.next(), MAXDEVICENAME,"Device name must be 15 or fewer characters");
}

// Convert a positive number in char string form to an int
// return the value or -1 if invalid numeric string
int chartoi(const char *s, int max){
  int result = 0;
  int l = strlen(s);
  for(int d = 0; d < l; d++){ 
    if(isdigit(s[d])){
      result = result * 10 + s[d] - '0';
    } else {
      result = -1;    // non-numeric char
      break;
    }
    if(result > max) {
      result = -1;
      break;
    }
  }
  return result;
}

// OneWire scan and setup
// Command format: wire port#
void wireCmd(){
  // Process optional first paramater - Port No.
  char portStr[3]; portStr[0]=0;
  if(processConfigParam(portStr,serCmd.next(),sizeof(portStr),"Port number must be 2 or fewer decimal digits") &&
         portStr[0] != 0){
    int port = chartoi(portStr,MAXPORTNO);
    if(port < 0){
      Serial.println("Invalid port number");
    } else {
      configData.owPort = port;
    }
  }
  Serial.print("Looking for OneWire devices on port:");
  Serial.println(configData.owPort);
  //See if any devices are responding
  //if(oneWire.reset() == 0){
    //No devices reporting or bus not operating
  //  Serial.println("No devices reporting");
  //}
  //populate the device index table
  oneWire.begin(configData.owPort);
  delay(250);
  oneWire.reset_search();
  delay(250);
  //find all the onewire devices
  DeviceAddress da;
  byte di = 0;
  while(oneWire.search(da)){
    Serial.print("Device:");
    Serial.print(di);
    Serial.print(" - ");
    for(byte i=0;i<8;i++){
      if(da[i]<16) Serial.print("0");
      Serial.print(da[i], HEX);
      }
    if(oneWire.crc8(da,7)!= da[7]){
      Serial.print(" CRC not valid!");
    }
    if(di<MAXOWDEVICES){
      for(byte i =0;i<8;i++) configData.owDevices[di][i] = da[i];
      Serial.println();
    } else {
      Serial.print("-more than ");
      Serial.print(MAXOWDEVICES);
      Serial.println(" Devices");
    }
    di++;
  }
  configData.owDevCount = di > MAXOWDEVICES ? MAXOWDEVICES : di;
  Serial.print(configData.owDevCount);
  Serial.println(" devices registered");
} 

// temp sensor (DS18B12) setup
// command format: temp interval repStyle
// where:
//      interval is the maximum reporting interval in seconds - 30-90 default is 60
//      repStyle is the number format - 0-2 default is 0
//        0 = integer(nnn) in deg C 
//        1 = integer(nnn) in 1/10 deg C
//        2 = decimal fraction (nn.n) deg C 
// all sensors are reported with the same interval and precision
// temperature is reported in degrees C
void tempCmd(){
  // Process optional first paramater - reporting interval
  char intvlStr[3]; intvlStr[0]=0;
  if(processConfigParam(intvlStr,serCmd.next(),sizeof(intvlStr),"Reporting interval must be 2 or fewer decimal digits") &&
         intvlStr[0] != 0){
    int intvl = chartoi(intvlStr,90);
    if(intvl < 30){
      Serial.println("Invalid reporting interval. Set to 60.");
      intvl = 60;
    }
    configData.reportIntvl = intvl;
    }
  char styleStr[4]; styleStr[0]=0;
  if(processConfigParam(styleStr,serCmd.next(),sizeof(styleStr),"Precision must be 1 decimal digit") &&
         intvlStr[0] != 0){
    int style = chartoi(styleStr,1);
    if(style < 0){
      Serial.println("Invalid precision. Set to 0.");
      style = 0;
    }
        configData.repStyle = style;
  }  
}

// Load Configuration Data from EEPROM
// Command format: load
void loadCmd(){
  if(configData.dirty){
    Serial.print("Config data has changed. Discard changes? (y/n)");
    while(Serial.available() < 1){
      int ch = Serial.read();
//      Serial.write(ch); Serial.println("");
      if(ch == 'y'){
        break;
      } else if(ch == 'n') {
        Serial.println(" - Config data not loaded");
        return;
      }
    }
  }
  Serial.println(" - Loading config data");
  // Load the configuration data from (pseudo) EEPROM
  EEPROM.begin(512);
  EEPROM.get(configAddr, configData);
}


// Save configuration data
// Save the config data to EEPROM if it has changed
// Command format: save
void saveCmd(){
  if(configData.dirty){
    configData.dirty = false;
    EEPROM.put(configAddr, configData);
  }
  if(EEPROM.commit()){    
    Serial.println("EEPROM write successful");
  } else {
    configData.dirty = true;     //write failed so still dirty
    Serial.println("EEPROM write failed");
  }
  
}

// Exit command mode and reboot
// Command format: exit
void exitCmd(){
  while(configData.dirty){
    Serial.print("Config data has changed. Save it? y/n:");
    
    while(Serial.available() < 1){}
    int ch = Serial.read();
//    Serial.write(ch); Serial.println("");
    if(ch == 'y'){
      saveCmd();
    } else if(ch == 'n') {break;}
  }
  ESP.restart();
}

//display config data command
#define P(P1,P2) {Serial.print(P1);Serial.write(':');Serial.println(P2);}
//inline void p(String p1, char *p2){Serial.print(p1);Serial.write(':');Serial.println(p2);}

void dispCmd(){
  
  P("WIFI SSID",configData.wifissid);
  P("WIFI Password",configData.wifipw);
  P("MQTT ip address",configData.mqttip);
  P("MQTT Password", configData.mqttpw);
  P("MQTT TL Topic", configData.mqtttltopic);
  P("Device Name",configData.devname);
  P("Reporting Interval",configData.reportIntvl);
  P("Reporting Value Style",configData.repStyle);

  P("OneWire Devices", configData.owDevCount);
    for(int di = 0; di < configData.owDevCount; di++){
    Serial.print("  Dev:");
    Serial.print(di);
    Serial.print(" ");
    for(byte i=0;i<8;i++){
      if(configData.owDevices[di][i]<16) Serial.print("0");
      Serial.print(configData.owDevices[di][i], HEX);
      }
    Serial.println();
  }
  P("One Wireport no", configData.owPort);
  P("Save required", configData.dirty);
}

// invalid command processor
void invCmd(const char *token){
  Serial.print("invalid command:");
  Serial.println(token);
}

void commandMode(){
  //Decide whether to enter configuration mode
  Serial.println("\n\nPress any key to enter configuration mode");
  
  //Wait 5 seconds for serial port to receive a charcter
  int config = 0;
  for (int i = 50; i > 0; i--)
  {
    config = Serial.available();
    if(config > 0) break;
    delay(100);
  };
  // If a character was received enter command mode
  // Otherwise return to normal setup sequence
  // Once in command mode the only way to exit
  // is to use the exit command which optionally
  // saves any changed configuration to EEPROM
  // and then restarts the processor. Alternatively
  // one can use a hardware reset which would
  // restart the processor without saving any
  // pending changes to configuration data.
  if(config > 0)
  {
    //Define available console commands
    serCmd.addCommand("wifi", wifiCmd);
    serCmd.addCommand("mqtt", mqttCmd);
    serCmd.addCommand("wire", wireCmd);
    serCmd.addCommand("temp", tempCmd);
    serCmd.addCommand("disp", dispCmd);
    serCmd.addCommand("save", saveCmd);
    serCmd.addCommand("load", loadCmd);
    serCmd.addCommand("exit", exitCmd);
    serCmd.setDefaultHandler(invCmd);

    //Load and display the current configuration
    loadCmd();
    dispCmd();
    
    // Read and process commands until processor is restarted
    while(true)
    {
      serCmd.readSerial();
    }
  }

}
///////////////////////////////
// Wifi configuration and setup
///////////////////////////////

void wifiSetup(char *ssid, char *pw){

  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  // Set the ESP8266 to be a WiFi-client
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  
   while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
   }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
///////////////////////////////
// MQTT Configuration and setup
///////////////////////////////

char mqttTopicP1[MAXMQTTTLTOPIC+MAXDEVICENAME+1];
#define ERRTOPIC "/err"
char mqttErrTopic[MAXMQTTTLTOPIC+MAXDEVICENAME+sizeof(ERRTOPIC)+1];
char deviceName[MAXDEVICENAME];
char topicBuf[256];


void psCallback(char *topic, byte *payload, unsigned int len){

}

void mqttSetup(const char *ipStr){
  // The lengths of the configuration strings have already been validated during 
  // command processing. The storage for the mqttTopicP1, mqttErrTopic and devicename above is
  // sized to accommodate the maximum string length permitted.
  strcpy(mqttTopicP1, configData.mqtttltopic);
  strcat(mqttTopicP1, "/");
  strcat(mqttTopicP1, configData.devname);
  strcpy(deviceName, configData.devname);
  strcpy(mqttErrTopic,mqttTopicP1);
  strcat(mqttErrTopic,ERRTOPIC);

  // IP Address has already been validated
  IPAddress ipadd;
  ipadd.fromString(ipStr);
  psClient.setServer(ipadd,1883);
  psClient.setCallback(psCallback);
}
//Publish a string message
void pubMsg(const char *topic, const char *msg){
  char topBuf[200];
  if(strlen(topic) < sizeof(topBuf)-sizeof(mqttTopicP1)-1){
    strcpy(topBuf, mqttTopicP1);
    strcat(topBuf, "/");
    strcat(topBuf, topic);
    psClient.publish(topBuf,msg);
  } else{
    psClient.publish(mqttErrTopic, topic);
  }  
}
//Publish an integer value message
void pubIntValue(const char *topic, int i, byte style){
  char istr[20];
  const char* fmt[3] = {"%0.0F","%0.0F","%0.1F"};

  float val = (float) i;
  
  if(style > 2){
    style = 1;
  } else {
    if(style == 0){
      val = val/10;
    }
  }
  
  sprintf(istr,fmt[style],val);
  pubMsg(topic, istr);
}

// connect and to mqtt server and reconnect when connection drops
void reconnect() {
  // Loop until we're reconnected
  char clientID[MAXDEVICENAME+4];
  while (!psClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    snprintf(clientID,size_t(clientID),"%s%04x",deviceName,(unsigned int)random(0xffff));
    Serial.print("with ID:");Serial.print(clientID);

    // Attempt to connect
    if (psClient.connect(clientID)) {
      Serial.println("...connected");

      // Once connected, publish an announcement...
      pubMsg("status","connected");

      // ... and resubscribe
      char inTopic[MAXMQTTTLTOPIC+MAXDEVICENAME+20];
      strcpy(inTopic, mqttTopicP1);
      strcat(inTopic, "/");
      strcat(inTopic,"reqstatus");
      psClient.subscribe(inTopic);
 
    } else {
      Serial.print("...failed, rc=");
      Serial.print(psClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

///////////////////////////////////
// One Wire Configuration and Setup
///////////////////////////////////

void owSetup(byte port){
  oneWire.begin(port);
}

/////////////////////////////////////////////////////
// Dallas Temperature Sensors Configuration and Setup
/////////////////////////////////////////////////////

#define SMOOTHCOUNT (4)
#define DISCONNECTED (DEVICE_DISCONNECTED_RAW)
// The intervals below are in seconds * .98 since the cycle clock ticks every 1024 ms.
#define DSMAXREPINTVL 59
#define DSMINREPINTVL 29
byte dsCount = 0;          // number of valid temperature devices found
byte tempStyle = 0;     // 0 or 1 fractional digits reported 0 means degrees C as an integer
                          //                                   1 means 1/10 degree C as an integer
byte rec_cycle = 0;       // recording cycle
int rep_intvl;  // max reporting interval - default 1 min in 10ths of a second
struct TempSensor{
  DeviceAddress dsad;       // one-wire device address of the ds18b20
  int tote;               // sum of the last SMOOTHCOUNT readings
  int reported;           // last reported tote value
  byte next;              // next readings slot to fill
  byte last_cycle;         // last cycle the sensor was reported
  int readings[SMOOTHCOUNT];  // last SMOOTHCOUNT readings
} sensors[MAXDEVICENAME];

void dispSensor(byte di){
  Serial.printf("\n[%d]Tote:%d Reported:%d", di, sensors[di].tote, sensors[di].reported);
  if(sensors[di].tote == DISCONNECTED)Serial.println("DISCONNECTED");
  Serial.write(ds.isConnected(sensors[di].dsad)? '^':'v');
  for(byte i=0;i<8;i++){
    if(sensors[di].dsad[i]<16) Serial.write('0');
    Serial.print(sensors[di].dsad[i], HEX);
  }
  Serial.printf("\nCycle:%d Next:%d\n", sensors[di].last_cycle, sensors[di].next);
  byte n= sensors[di].next;
  for(byte i=0; i<SMOOTHCOUNT; i++){
    Serial.print(i == n ? " *" : " ");
    Serial.print(sensors[di].readings[i]);
  }
  Serial.println();
}

// Sensor reading and reporting task list and scheduler

enum {
  DSTRIGGERTASK,    // Triggers all sensors to store current temp value
  DSXFERTASK,    // Transfers temps from each sensor to memory
  DSPUBTASK,     // Publish all temps outstanding
  DSREPORTTASK,    // Force reporting of unchanged sensors
  TASKLIMIT
  } ;
word schedlist[TASKLIMIT];
Scheduler scheduler(schedlist,TASKLIMIT);



void sensorSetOffline(int si){
  sensors[si].tote = DISCONNECTED;
  sensors[si].next = 0;
  dispSensor(si);
}

// Round & scale a reading
inline int rosc(int reading){
  return( (reading + (reading < 0 ? -SMOOTHCOUNT : SMOOTHCOUNT)/2)/SMOOTHCOUNT );
}

void sensorSetOnline(int si, int reading){
  
  int rdg = rosc(reading);
  for(int i = 0; i < SMOOTHCOUNT; i++){
    sensors[si].readings[i] = rdg;
  }
  sensors[si].tote = rdg * SMOOTHCOUNT;
  sensors[si].next = 0;

  dispSensor(si);
}

void sensorSetup(int dc){
  ds.begin();
  ds.setWaitForConversion(false);
  tempStyle = configData.repStyle;
  rep_intvl = configData.reportIntvl;

  int dsc = 0;
  for(int di = 0; di<dc; di++){
    //only process DS temperature sensors
    if(ds.validFamily(configData.owDevices[di])){
      // copy device name to sensor table
      for(byte i = 0; i < sizeof(DeviceAddress); i++){
        sensors[dsc].dsad[i] = configData.owDevices[di][i];
      }
    // starts offline and turns online when a valid temperature is received
    sensorSetOffline(dsc);
    dsc++;  
    }
  } 
  dsCount= dsc;
  scheduler.timer(DSTRIGGERTASK,50); // schedule the start of temp reporting 5 seconds after startup
}

// Trigger all DS temperature devices to read and store temp values
void dsTrigger(){
  ds.requestTemperatures();
  scheduler.timer(DSTRIGGERTASK, 150);         // schedule to run again in 15 Seconds
  scheduler.timer(DSXFERTASK, 12);             // allow ~1 sec before xfer of temp values 
}

// transfer temp values to and update records for each sensor
void dsXfer(){
  for(byte di=0; di < dsCount; di++){
    int newReading = ds.getTemp(sensors[di].dsad);
    if(newReading != DISCONNECTED){
      // sensor has reported a good value
      if(sensors[di].tote != DISCONNECTED){   // did the sensor previously report itself as DISCONNECTED
        // NO: normal path sensor is continuing to report good values
        // update the tote and save the new reading
        newReading = rosc(newReading);
        sensors[di].tote -= sensors[di].readings[sensors[di].next];
        sensors[di].readings[sensors[di].next] = newReading;
        sensors[di].tote += newReading;
        //update the pointer to the new oldest reading
        if(++sensors[di].next >= SMOOTHCOUNT) sensors[di].next = 0;
      } else {
        // sensor has started reporting good values
        sensorSetOnline(di,newReading);
      }
    } else {
      // sensor did not report a good value
      if(sensors[di].tote != DISCONNECTED)
      // sensor has just changed from previously reporting good values
      sensorSetOffline(di);
    }
    dispSensor(di);
  }
scheduler.timer(DSPUBTASK, 0); // schedule sensor value pub task to run
}

// publish all values due for publishing
// all changed values are published subject to the min reporting interval
// unchanged values are reported periodically in accordance with
// the max reporting interval
void dsPub(){
  // determine where we are in the wrap-around 256 second cycle
  byte now = (millis() >> 10) & 0xFF;

  for( byte di = 0; di < dsCount; di++){
    byte elapsed = now - sensors[di].last_cycle;
    if (elapsed < rep_intvl){
      // has been reported within max interval so only report if changed
      if((elapsed < DSMINREPINTVL) || (sensors[di].tote == sensors[di].reported)){
        //Sensor last reported too recently or hasn't changed
        break;
      }
    }
    // Report the sensor value
    // Temp values are stored in units of 1/128 degree C
    // Convert to integer value in units of 1/10 degree C rounded to nearest 1/10 degree
    int degC = (int) (((long)sensors[di].tote * 10 + 64)/128);

    // publish the temperature value
    pubIntValue("reading/temp", degC, tempStyle);
    Serial.printf("pub ds%d T=%d at %d Tote=%d Rep=%d\n",di,degC,now,sensors[di].tote, sensors[di].reported);

    sensors[di].reported = sensors[di].tote;
    sensors[di].last_cycle = now;

  }
}

void SensorScan(void){
  switch(scheduler.poll()){
    case DSTRIGGERTASK:
      dsTrigger();
      break;
    case DSXFERTASK:
      dsXfer();
      break;
    case DSPUBTASK:
      dsPub();
      break;
    case DSREPORTTASK:
      break;
    default:
    // Report error?
      break;
  }
}


void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  
  commandMode();        // enter command mode if operator chooses to do so

  loadCmd();            // load the configuration from EEPROM
  dispCmd();            // display the current configuration

  wifiSetup(configData.wifissid, configData.wifipw);  // set up wifi and connect to network
 
  randomSeed(micros());     // wifi setup delay allows a stochastic seed for random number generation

  mqttSetup(configData.mqttip);         // setup MQTT configuration and connect to pub/sub server

  owSetup(configData.owPort);      // Enable Onewire operation

  sensorSetup(configData.owDevCount); // Identify 1-wire Temp sensors and set up reporting

Serial.println("Starting operation");
}

#define TOPIC_BUFFER_SIZE 64
char topic[TOPIC_BUFFER_SIZE];
#define MSG_BUFFER_SIZE 200
char msg[MSG_BUFFER_SIZE];
unsigned long lastMsg=0;
unsigned long secs = 0;


void loop() {
  if (!psClient.connected()) {
    reconnect();
  }
  psClient.loop();
  unsigned long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg += 10000;
    secs += 10;
    snprintf (msg, MSG_BUFFER_SIZE, "Up time: %ld seconds", secs);
    Serial.print("Publish message: ");
    Serial.println(msg);
    pubMsg("status/uptime", msg);
  }
  // Check all sensors and publish readings as required
  SensorScan();
}

