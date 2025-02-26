/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <WiFiManager.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESPUI.h>
#include <ESPmDNS.h>
// #include <DNSServer.h>
#include <Adafruit_MCP23008.h>
#include <ctime>
#include <EEPROM.h>
#include <http_ota.h>


Adafruit_MCP23008 mcp;
WiFiManager wifiManager;

// #define HTTP_GET ESP_HTTP_GET
// #define HTTP_POST ESP_HTTP_POST
// #define HTTP_DELETE ESP_HTTP_DELETE
// #define HTTP_PUT ESP_HTTP_PUT
// #define HTTP_OPTIONS ESP_HTTP_OPTIONS
// #define HTTP_PATCH ESP_HTTP_PATCH

String deviceToken, hostUI;

unsigned long previousMill = 0;
unsigned long currentMill = 0;

unsigned long updateLog = 0;
unsigned long currentOTA = 0;

time_t lastEpoch = 0;
int count(0);
// const char* relay = "{\"relay1\":0, \"relay2\":1, \"relay3\":2, \"relay\":3}";
int relay[] = {0, 1, 2, 3};

char macStr[18];



// Structure example to receive data
// Must match the sender structure
typedef struct pms7003data
{
  int id;
  uint16_t temp, hum, pres, atm;
  uint16_t pm01_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t TVOC, eCO2;
  String boradMAC, option, passCode;
  uint16_t set, button, verti, resWiFi;
  String freshStat, f1Stat, f2Stat = "0";

  //GUI
  uint16_t idLabel, roleLabel, sender, stamp, FAG, fan1, fan2, maxPM, maxCO2, setDetail, fanMode, fanSwitch;

  // int tm_hour, tm_min, tm_sec;
  int sum[4] = {};
  int avg[4] = {};

  uint16_t pmSet, co2Set;
  String fresh, f1, f2, mode;
  char sendMode[15];

  int otaTime;

} pms7003data;

pms7003data data; //for pure data var
pms7003data ui1; //for GUI var
pms7003data ui2; //for GUI var
// pms7003data ui3; //for GUI var
pms7003data GUI;

struct tm timeInfo;

// Create a structure to hold the readings from each board
pms7003data board1;
pms7003data board2;
// pms7003data board3;

// // Create an array with all the structures
pms7003data boardsStruct[3] = {board1, board2};
pms7003data UI[3] = {ui1, ui2};

struct tcp_pcb;
extern struct tcp_pcb* tcp_tw_pcbs;
extern "C" void tcp_abort(struct tcp_pcb* pcb);

String getFormattedDateTime();
void updateUI();
void enterSettingCallback(Control *sender, int type);
void writeEEPROM();
void controlRelay();
void enterRSTCallback(Control *sender, int type);

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  count++;
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&data, incomingData, sizeof(data));
  // memcpy(&sw, incomingData, sizeof(sw));
  Serial.printf("Board ID %u: %u bytes\n", data.id, len);
  // Update the structures with the new incoming data
  boardsStruct[data.id-1].id = data.id;
  boardsStruct[data.id-1].boradMAC = macStr;
  boardsStruct[data.id-1].temp = data.temp;
  boardsStruct[data.id-1].hum = data.hum;
  boardsStruct[data.id-1].pres = data.pres;
  boardsStruct[data.id-1].atm = data.atm;
  boardsStruct[data.id-1].pm01_env = data.pm01_env;
  boardsStruct[data.id-1].pm25_env = data.pm25_env;
  boardsStruct[data.id-1].pm100_env = data.pm100_env;
  boardsStruct[data.id-1].particles_03um = data.particles_03um;
  boardsStruct[data.id-1].particles_05um = data.particles_05um;
  boardsStruct[data.id-1].particles_10um = data.particles_10um;
  boardsStruct[data.id-1].particles_25um = data.particles_25um;
  boardsStruct[data.id-1].particles_50um = data.particles_50um;
  boardsStruct[data.id-1].particles_100um = data.particles_100um;
  boardsStruct[data.id-1].TVOC = data.TVOC;
  boardsStruct[data.id-1].eCO2 = data.eCO2;
  boardsStruct[data.id-1].option = String(data.sendMode);
  boardsStruct[data.id-1].maxPM = data.pmSet;
  boardsStruct[data.id-1].maxCO2 = data.co2Set;
  
  // Serial.printf("id value: %d \n", data.id);
  // Serial.printf("temp value: %d \n", data.temp);
  // Serial.printf("hum value: %d \n", data.hum);
  // Serial.printf("pres value: %d \n", data.pres);
  // Serial.printf("atm value: %d \n", data.atm);
  // Serial.printf("pm01_env value: %d \n", data.pm01_env);
  // Serial.printf("pm25_env value: %d \n", data.pm25_env);
  // Serial.printf("pm100_env value: %d \n", data.pm100_env);
  // Serial.printf("particles_03um value: %d \n", data.particles_03um);
  // Serial.printf("particles_05um value: %d \n", data.particles_05um);
  // Serial.printf("particles_10um value: %d \n", data.particles_10um);
  // Serial.printf("particles_25um value: %d \n", data.particles_25um);
  // Serial.printf("particles_50um value: %d \n", data.particles_50um);
  // Serial.printf("particles_100um value: %d \n", data.particles_100um);
  // Serial.printf("TVOC value: %d \n", data.TVOC);
  // Serial.printf("eCO2 value: %d \n", data.eCO2);
  // Serial.println("debug Rev");
  // Serial.printf("MODE: %s \n", data.option);
  // Serial.println();
  data.option = boardsStruct[data.id-1].option;
  data.maxPM = boardsStruct[data.id-1].pmSet;
  data.maxCO2 = boardsStruct[data.id-1].co2Set;

  Serial.printf("MODE: %s \n", data.option.c_str());

  data.sum[0] += boardsStruct[0].pm25_env;
  data.sum[1] += boardsStruct[1].pm25_env;
  data.sum[2] += boardsStruct[0].eCO2;
  data.sum[3] += boardsStruct[1].eCO2;
  if (count == 5){
    for (int i = 0; i < 4; i++){
      data.avg[i] = data.sum[i] / count;
    }
    // if (avg > data.maxPM || avg > data.maxCO2){
    //   for (int i = 0; i < 3; i++){
    //     mcp.digitalWrite(relay[i], HIGH);
    //   }
    // } else if (avg < data.maxPM || avg < data.maxCO2) {
    //   for (int i = 0; i < 3; i++)
    //   {
    //     mcp.digitalWrite(relay[i], LOW);
    //   }
      
    // }
    // controlRelay();
  }
}

void updateUI(){
  Serial.println("debug update 1 ");
  // Serial.println("sizeof(UI): " + String(sizeof(UI)));
  ESPUI.updateLabel(GUI.fanMode, String(data.option));
  for (int i = 0; i < 2; i++){
    // Serial.println("debug update 2 ");
    ESPUI.updateLabel(UI[i].id, String(boardsStruct[i].id));
    ESPUI.updateLabel(UI[i].pm01_env, String(boardsStruct[i].pm01_env));
    ESPUI.updateLabel(UI[i].pm25_env, String(boardsStruct[i].pm25_env));
    ESPUI.updateLabel(UI[i].pm100_env, String(boardsStruct[i].pm100_env));
    ESPUI.updateLabel(UI[i].eCO2, String(boardsStruct[i].eCO2));
    ESPUI.updateLabel(UI[i].sender, String(boardsStruct[i].boradMAC));
    // Serial.println("debug update 3 ");
    ESPUI.updateLabel(UI[i].stamp, String(getFormattedDateTime()));

    Serial.printf("id value: %d \n", boardsStruct[i].id);
    Serial.printf("temp value: %d \n", boardsStruct[i].temp);
    Serial.printf("hum value: %d \n", boardsStruct[i].hum);
    Serial.printf("pres value: %d \n", boardsStruct[i].pres);
    Serial.printf("atm value: %d \n", boardsStruct[i].atm);
    Serial.printf("pm01_env value: %d \n", boardsStruct[i].pm01_env);
    Serial.printf("pm25_env value: %d \n", boardsStruct[i].pm25_env);
    Serial.printf("pm100_env value: %d \n", boardsStruct[i].pm100_env);
    Serial.printf("particles_03um value: %d \n", boardsStruct[i].particles_03um);
    Serial.printf("particles_05um value: %d \n", boardsStruct[i].particles_05um);
    Serial.printf("particles_10um value: %d \n", boardsStruct[i].particles_10um);
    Serial.printf("particles_25um value: %d \n", boardsStruct[i].particles_25um);
    Serial.printf("particles_50um value: %d \n", boardsStruct[i].particles_50um);
    Serial.printf("particles_100um value: %d \n", boardsStruct[i].particles_100um);
    Serial.printf("TVOC value: %d \n", boardsStruct[i].TVOC);
    Serial.printf("eCO2 value: %d \n", boardsStruct[i].eCO2);
    Serial.println();
  }
  
  Serial.println("time = " + getFormattedDateTime());
}


void getMac()
{
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.println("OK");
  Serial.print("+deviceToken: ");
  Serial.println(WiFi.macAddress());
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) {
      deviceToken += "0"; // Add leading zero if needed
    }
    deviceToken += String(mac[i], HEX); // Convert byte to hex
  }
  deviceToken.toUpperCase();
  ESPUI.updateLabel(GUI.idLabel, String(deviceToken));
}

void tcpCleanup(void) {
  Serial.println("Debug TCPclean()");
    while (tcp_tw_pcbs) {
        tcp_abort(tcp_tw_pcbs);
    }
}

void _initUI(){
  // Serial.println("Start Debug init UI");
  hostUI = "SchoolReceiver:" + deviceToken;
  //host2 = "SmartPeir";
    // if (WiFi.status() == WL_CONNECTED) {
    //  Serial.println(WiFi.localIP());
    //  Serial.println("Wifi started");

    //  if (!MDNS.begin(hostUI.c_str())) {
    //    Serial.println("Error setting up MDNS responder!");
    //  }
    // } else {
		Serial.println("\nCreating access point...");
    Serial.println(WiFi.localIP());
		WiFi.mode(WIFI_AP_STA); // Correct
		WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
		WiFi.softAP(hostUI.c_str(), "green7650");
    Serial.print("SSID: ");
    Serial.println(hostUI);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("Wi-Fi Mode: ");
    Serial.println(WiFi.getMode()); // Should print 2 for AP mode

  // }
  // Serial.println("End Debug init UI");
  
}

void setUpUI()
{
  tcpCleanup();
  // Turn off verbose  ging
  ESPUI.setVerbosity(Verbosity::Quiet);

  // Make sliders continually report their position as they are being dragged.
  ESPUI.sliderContinuous = true;

  // This GUI is going to be a tabbed GUI, so we are adding most controls using ESPUI.addControl
  // which allows us to set a parent control. If we didn't need tabs we could use the simpler add
  // functions like:
  //     ESPUI.button()
  //     ESPUI.label()

  /*
     Tab: Basic Controls
     This tab contains all the basic ESPUI controls, and shows how to read and update them at runtime.
    -----------------------------------------------------------------------------------------------------------*/
  auto maintab = ESPUI.addControl(Tab, "", "Home");
  // nameLabel = ESPUI.addControl(Label, "Device Name", "AIRMASS 2.5 Inspector", Emerald, maintab);
  GUI.roleLabel = ESPUI.addControl(Label, "Device Role:", "Receiver", Emerald, maintab);
  GUI.idLabel = ESPUI.addControl(Label, "Mac Address:", String(deviceToken), Emerald, maintab);
  // firmwarelabel = ESPUI.addControl(Label, "Firmware", String(current_version), Emerald, maintab);
  
  // GUI.set = ESPUI.addControl(Tab, "", "Setting");
  // GUI.mode = ESPUI.addControl(Select, "Mode Control Selection", "Please Select Mode", Alizarin, GUI.set, enterSettingCallback);
  // ESPUI.addControl(Option, "", "", Emerald, GUI.mode, enterSettingCallback);
  // ESPUI.addControl(Option, "Automatic Mode", "auto", Emerald, GUI.mode, enterSettingCallback);
  // ESPUI.addControl(Option, "Manual Mode", "manual", Emerald, GUI.mode, enterSettingCallback);

  auto setting = ESPUI.addControl(Tab, "", "Setting");
  GUI.otaTime = ESPUI.addControl(Number, "OTA Interval", String(data.otaTime), Emerald, setting, enterSettingCallback);
  ESPUI.addControl(Button, "", "SAVE", None, setting, enterSettingCallback);


  auto log =ESPUI.addControl(Tab, "", "AIRMASS 1");
  UI[0].stamp = ESPUI.addControl(Label, "Timestamp:", getFormattedDateTime(), Emerald, log);
  UI[0].sender = ESPUI.addControl(Label, "received from", String(boardsStruct[0].boradMAC), Emerald, log);
  UI[0].id = ESPUI.addControl(Label, "Board ID", String(boardsStruct[0].id), Emerald, log);
  UI[0].pm01_env = ESPUI.addControl(Label, "PM 1.0:", String(boardsStruct[0].pm01_env), Emerald, log);
  UI[0].pm25_env = ESPUI.addControl(Label, "PM 2.5:", String(boardsStruct[0].pm25_env), Emerald, log);
  UI[0].pm100_env = ESPUI.addControl(Label, "PM 10:", String(boardsStruct[0].pm100_env), Emerald, log);
  UI[0].eCO2 = ESPUI.addControl(Label, "Carbon dioxide (CO2)", String(boardsStruct[0].eCO2), Emerald, log);

  auto log1 =ESPUI.addControl(Tab, "", "AIRMASS 2");
  UI[1].stamp = ESPUI.addControl(Label, "Timestamp:", getFormattedDateTime(), Emerald, log1);
  UI[1].sender = ESPUI.addControl(Label, "received from", String(boardsStruct[1].boradMAC), Emerald, log1);
  UI[1].id = ESPUI.addControl(Label, "Board ID", String(boardsStruct[1].id), Emerald, log1);
  UI[1].pm01_env = ESPUI.addControl(Label, "PM 1.0:", String(boardsStruct[1].pm01_env), Emerald, log1);
  UI[1].pm25_env = ESPUI.addControl(Label, "PM 2.5:", String(boardsStruct[1].pm25_env), Emerald, log1);
  UI[1].pm100_env = ESPUI.addControl(Label, "PM 10:", String(boardsStruct[1].pm100_env), Emerald, log1);
  UI[1].eCO2 = ESPUI.addControl(Label, "Carbon dioxide (CO2)", String(boardsStruct[1].eCO2), Emerald, log1);

  auto debug = ESPUI.addControl(Tab, "", "Debug");
  GUI.fanMode = ESPUI.addControl(Label, "Mode", String(data.option), Alizarin, debug);

  auto connect = ESPUI.addControl(Tab, "", "Connection");
  GUI.resWiFi = ESPUI.addControl(Text, "Restart WiFi", String(data.passCode), Alizarin, connect, enterRSTCallback);
  ESPUI.addControl(Button, "RESTART", "RST", Peterriver, connect, enterRSTCallback);

  ESPUI.begin(hostUI.c_str());
}

// void enterAutoCallback(Control *sender, int type){
//   Serial.println(sender->value);
//   ESPUI.updateControl(sender);

//   if (type == B_UP)
//   {
//     Control* _pm = ESPUI.getControl(GUI.maxPM);
//     Control* _co2 = ESPUI.getControl(GUI.maxCO2);
//     data.maxPM = _pm->value.toInt();
//     data.maxCO2 = _co2->value.toInt();
//     writeEEPROM();
//   }
// }

void enterSettingCallback(Control* sender, int type) {
  Serial.println(sender->value);
  ESPUI.updateControl(sender);

  if (type == B_UP)
  {
    data.otaTime = ESPUI.getControl(GUI.otaTime)->value.toInt();
    writeEEPROM();
  }

    
}

void enterRSTCallback(Control *sender, int type){
  Serial.println(sender->value);
  // ESPUI.updateControl(sender);
  Control* _rst = ESPUI.getControl(GUI.resWiFi);
  data.passCode = _rst->value.c_str();

  if (type == B_UP) { // Only trigger on button release
    if (data.passCode.equals("systemctl restart networking")) {  
      Serial.println("Restarting WiFi...");
      
      // Reset WiFi credentials
      wifiManager.resetSettings();  
      
      // Restart ESP
      ESP.restart();  
    } else {
      Serial.println("Invalid command received.");
    }
  }
}

void _initRelay() {
  mcp.begin();      // use default address 0

  //test section
  for (int i = 0; i < 4; i++)
  {
    mcp.pinMode(relay[i], OUTPUT);
    mcp.digitalWrite(relay[i], HIGH);
    Serial.println("ON");
    delayMicroseconds(1000000);
  }
  for (int i = 3; i > -1; i--)
  {
    mcp.digitalWrite(relay[i], LOW);
    Serial.println("OFF");
    delayMicroseconds(1000000);
  }
}

void controlRelay(){
    if ((data.avg[0] > data.maxPM && data.avg[1] > data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] > data.maxCO2)
        || (data.avg[0] > data.maxPM && data.avg[1] < data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] > data.maxCO2)
        || (data.avg[0] < data.maxPM && data.avg[1] > data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] > data.maxCO2)
        || (data.avg[0] < data.maxPM && data.avg[1] < data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] > data.maxCO2)){ //1

      for (int i = 0; i < 3; i++){
        mcp.pinMode(relay[i], HIGH);
      }

    }else if ((data.avg[0] > data.maxPM && data.avg[1] > data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] > data.maxCO2)
              || (data.avg[0] < data.maxPM && data.avg[1] < data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] > data.maxCO2)
              || (data.avg[0] > data.maxPM && data.avg[1] < data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] > data.maxCO2)
              || (data.avg[0] < data.maxPM && data.avg[1] > data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] > data.maxCO2)){ //2

      for (int i = 0; i < 3; i++){
        if (i == 1) mcp.pinMode(relay[i], LOW); continue;
        mcp.pinMode(relay[i], HIGH);
      }

    }else if ((data.avg[0] > data.maxPM && data.avg[1] > data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] < data.maxCO2)
              || (data.avg[0] < data.maxPM && data.avg[1] < data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] < data.maxCO2)
              || (data.avg[0] > data.maxPM && data.avg[1] < data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] < data.maxCO2)
              || (data.avg[0] < data.maxPM && data.avg[1] > data.maxPM && data.avg[2] > data.maxCO2 && data.avg[3] < data.maxCO2)){

      for (int i = 0; i < 3; i++){
        if (i == 2) mcp.pinMode(relay[i], LOW); continue;
        mcp.pinMode(relay[i], HIGH);
      }
      
    }
    
    else if ((data.avg[0] < data.maxPM && data.avg[1] > data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] < data.maxCO2)
            || (data.avg[0] > data.maxPM && data.avg[1] < data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] < data.maxCO2)
            || (data.avg[0] > data.maxPM && data.avg[1] > data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] < data.maxCO2)) 
    {

      for (int i = 0; i < 3; i++){
        if (i == 1) mcp.pinMode(relay[i], HIGH); continue;
        mcp.pinMode(relay[i], LOW);
      }

    }

    else if (data.avg[0] < data.maxPM && data.avg[1] < data.maxPM && data.avg[2] < data.maxCO2 && data.avg[3] < data.maxCO2) 
    {

      for (int i = 0; i < 3; i++){
        mcp.pinMode(relay[i], LOW);
      }

    }
}

String getFormattedDateTime() {
  // Calculate elapsed time since the last sync
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - previousMill;  // Time elapsed since last sync
  unsigned long totalMillis = (lastEpoch * 1000) + elapsedMillis;  // Total time in milliseconds

  // Convert to seconds and adjust for timezone
  time_t currentTime = (totalMillis / 1000);  // Adjust for timezone
  unsigned long currentMillisPart = totalMillis % 1000; // Remaining milliseconds

  // Update `timeInfo` with adjusted local time
  localtime_r(&currentTime, &timeInfo);

  // Format hour, minute, and second with leading zeros (AFTER updating timeInfo)
  String hourStr = (timeInfo.tm_hour < 10 ? "0" : "") + String(timeInfo.tm_hour);
  String minStr = (timeInfo.tm_min < 10 ? "0" : "") + String(timeInfo.tm_min);
  String secStr = (timeInfo.tm_sec < 10 ? "0" : "") + String(timeInfo.tm_sec);

  char formattedTime[20];
  snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d",
           timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  return String(formattedTime);
}

void writeEEPROM(){
  int addr = 0;
  // EEPROM.put(addr, data.maxPM);
  // addr += sizeof(data.maxPM);
  // EEPROM.put(addr, data.maxCO2);
  // addr += sizeof(data.maxCO2);
  EEPROM.put(addr, data.otaTime);
  addr += sizeof(data.otaTime);

  EEPROM.commit();
}

void readEEPROM(){
  int addr = 0;
  // EEPROM.get(addr, data.maxPM);
  // addr += sizeof(data.maxPM);
  // EEPROM.get(addr, data.maxCO2);
  // addr += sizeof(data.maxCO2);

  // ESPUI.updateNumber(GUI.maxPM, data.maxPM);
  // ESPUI.updateNumber(GUI.maxCO2, data.maxCO2);

  EEPROM.get(addr, data.otaTime);
  addr += sizeof(data.otaTime);

  ESPUI.updateNumber(GUI.otaTime, data.otaTime);

}

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void _initWiFi(){
  // WiFi.mode(WIFI_AP_STA); // Correct
  String host = "QCM-" + deviceToken;
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(60); // auto close configportal after n seconds
  wifiManager.setAPClientCheck(true);     // avoid timeout if client connected to softap
  wifiManager.setBreakAfterConfig(true);  // always exit configportal even if wifi save fails
  if (!wifiManager.autoConnect(host.c_str()))
    {
      Serial.println("failed to connect and hit timeout");
      delay(1000);
    }
}

void setup() {
  Project = "BMA";
  FirmwareVer = "0.0.1";

  //Initialize Serial Monitor
  Serial.begin(115200);
  EEPROM.begin(100);
  readEEPROM();
  getMac();

  _initWiFi();

  _initUI();
  setUpUI();
  _initRelay();
  
  //Set device as a Wi-Fi Station
  // WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
  previousMill = millis();
  if (previousMill - updateLog >= 60000){
    updateLog = previousMill;
    // Serial.println("debug loop 1 ");
    updateUI();
    // Serial.println("debug loop 2 ");
    
  }

  if (previousMill - currentOTA >= data.otaTime * 1000) currentOTA = previousMill; OTA_git_CALL();

  if (data.option.equals("Automatic Mode")) controlRelay();

}