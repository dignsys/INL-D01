/* 
 * INL-D01 Main 
 * Author : DIGNSYS Inc.
 */

#include <Arduino.h>
#include <Wire.h>
#include <ICM42670P.h>
#include <Adafruit_SHT4x.h>
#include <DFRobot_TMF8x01.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEBeacon.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#define VERSION_INL_D01_FW  "20240103"

#define PIN_BOOT            0
#define PIN_BAT             2
#define PIN_PT              5
#define PIN_BUTTON          6
#define PIN_BUZZER          7
#define PIN_MQ2_AO          4

#define TMF8801_EN       -1                      //EN pin of of TMF8x01 module is floating, not used in this demo
#define TMF8801_INT      -1                      //INT pin of of TMF8x01 module is floating, not used in this demo

// Instantiate an ICM42670P with LSB address set to 0
ICM42670P IMU(Wire,0);

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

DFRobot_TMF8801 tof(/*enPin =*/TMF8801_EN,/*intPin=*/TMF8801_INT);

uint8_t caliDataBuf[14] = {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};//The 14 bytes calibration data which you can get by calibration.ino demo.

#define ENABLE_BLE_OP     1
#define BLE_FORMAT_TEST   0
#define ENABLE_MQTT_OP    0
#define SINGLE_TASK_PERIOD  250 // 20 tasks * 250 msec = 5 sec (full task period)
//#define INL_NOT_USING_SENSOR

#define DEVICE_NAME            "ESP32"
#define SERVICE_UUID           "7A0247E7-8E88-409B-A959-AB5092DDB03E"
#define BEACON_UUID            "2D7A9F0C-E0E8-4CC9-A71B-A21DB2D034A1"
#define BEACON_UUID_REV        "A134D0B2-1DA2-1BA7-C94C-E8E00C9F7A2D"
#define CHARACTERISTIC_UUID    "82258BAA-DF72-47E8-99BC-B73D7ECD08A5"

#if 0
char custom_beacon_uuid[37] = {
  'A','1','3','4','D','0','B','2','-',
  '1','D','A','2','-','1','B','A','7','-',
  'C','9','4','C','-','E','8','E','0','0','C','9','F','7','A','2','D', 0
};
#else
char custom_beacon_uuid[37] = {
  'A','A','A','A','A','A','A','A','-',
  'A','A','A','A','-','A','A','A','A','-',
  'A','A','A','A','-','A','A','A','A','A','A','A','A','A','A','A','A', 0
};
#endif
uint8_t custom_uuid_byte_0 = 0;

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t value = 0;

static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};

uint16_t st_count = 0;
uint8_t gv_flame = 0;
int16_t gv_axis_x = 0;
int16_t gv_axis_y = 0;
int16_t gv_axis_z = 0;
int16_t gv_gyro_x = 0;
int16_t gv_gyro_y = 0;
int16_t gv_gyro_z = 0;
uint16_t gv_gas = 0;
int16_t gv_temperature = 0; // x100
uint16_t gv_humidity = 0; // x100
uint16_t gv_illuminance = 0;
uint16_t gv_tof = 0;
float gvf_acc_x = 0.;
float gvf_acc_y = 0.;
float gvf_acc_z = 0.;
float gvf_gyro_x = 0.;
float gvf_gyro_y = 0.;
float gvf_gyro_z = 0.;
float gvf_temperature = 0.;
float gvf_humidity = 0.;
int gvi_flame = 0.;
int gvi_gas = 0;
float gvf_tof = 0.;

void init_service(void);
void init_beacon(void);
void update_beacon(void);
void update_beacon_a(void);
void update_beacon_b(void);
int get_char(uint8_t in_byte, char* pout_bytes);
bool checkCRC(const uint8_t *buf, uint16_t len);
void setCRC(uint8_t *buf, uint16_t len);
uint16_t CRC16(const uint8_t *data, uint16_t len);

class MyServerCallbacks: public BLEServerCallbacks {
  
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("deviceConnected = true");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("deviceConnected = false");

      // Restart advertising to be visible and connectable again
      BLEAdvertising* pAdvertising;
      pAdvertising = pServer->getAdvertising();
      pAdvertising->start();
      Serial.println("iBeacon advertising restarted");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }
        Serial.println();
        Serial.println("*********");

      }
    }
};

#define UG_BUTTON_DEBOUNCE    200 // 200: good, 50: initial

enum {
  UG_BUTTON_RELEASED,
  UG_BUTTON_PRESSED,
};

enum {
  STATE_BUZZER_OFF,
  STATE_BUZZER_ON,
};

int ug_button_isr_detected = 0;
int ug_button_status = UG_BUTTON_PRESSED;
unsigned long last_isr_millis = 0;

//#define MQ2_WARM_UP_TIME    300000  //1000*60*5  // 5 minutes
#define MQ2_WARM_UP_TIME    10000  //1000*10  // 10 seconds

int mq2_warm_up_end = 0;
unsigned long sys_start_millis = 0;

enum {
  STATE_FLAME_NONE,
  STATE_FLAME_DETECTED,
};

uint32_t crc32_value = 0xFFFFFFFF;
static const uint32_t crc32_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};
uint32_t crc32_state = 0;

const char* ssid = "DIGNSYS LAB 2G"; // WiFi SSID
const char* password = "***"; // WiFi Password
const char* mqttServer = "broker.emqx.io";
const int mqtt_port = 1883;
char mqtt_clientId[50];
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

#define EEPROM_SIZE 512
int eeprom_addr = 0;
byte eeprom_write_data;
byte eeprom_read_data;

#define EEPROM_ADDR_SN_NUM  16
#define EEPROM_ADDR_SSID    32
#define EEPROM_ADDR_PASSWD  64
#define EEPROM_ADDR_SERVER  96
#define EEPROM_ADDR_SN_NUM_SZ  5
#define EEPROM_ADDR_SSID_SZ    32
#define EEPROM_ADDR_PASSWD_SZ  32
#define EEPROM_ADDR_SERVER_SZ  32
uint8_t gv_id[EEPROM_ADDR_SN_NUM_SZ] = {0,};
char gv_ssid[EEPROM_ADDR_SSID_SZ] = {0,};
char gv_passwd[EEPROM_ADDR_PASSWD_SZ] = {0,};
char gv_server[EEPROM_ADDR_SERVER_SZ] = {0,};
uint16_t gv_address = 0x0001;
uint8_t gv_location = 0x01;
void get_eeprom_data(void);
void set_eeprom_data_sn_num(void);
void set_eeprom_data_ssid(void);
void set_eeprom_data_passwd(void);
void set_eeprom_data_server(void);
uint8_t wifi_connected = 0;

Preferences prefs;

typedef struct {
  uint8_t gv_id[EEPROM_ADDR_SN_NUM_SZ];
  char gv_ssid[EEPROM_ADDR_SSID_SZ];
  char gv_passwd[EEPROM_ADDR_PASSWD_SZ];
  char gv_server[EEPROM_ADDR_SERVER_SZ];
} settings_t;

settings_t gv_settings;

void connectToMQTT(void);
void mqtt_callback(char* topic, byte* message, unsigned int length);

// put function declarations here:
void sub_test_a(void);  // I2C Read Test
void sub_test_b(void);  // I2C Write Test
void sub_test_c(void);  // ICM42670P Test
void sub_test_d(void);  // Button Test
void sub_test_e(void);  // VEML3235 Test
void sub_test_g(void);  // SHT4x Test
void sub_test_h(void);  // TMF8801 Test
void sub_test_s(void);  // LittleFS Format Test
void sub_test_t(void);  // L-51POPT1D2 Test
void sub_test_u(void);  // Buzzer Test
void sub_test_v(void);  // BLE iBeacon Test
void sub_test_w(void);  // Settings
void sub_test_loop(void);

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_read_wo(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);

void sub_task_flame(void);
void sub_task_three_axis(void);
void sub_task_gas(void);
void sub_task_temperature(void);
void sub_task_illuminance(void);
void sub_task_tof(void);
void sub_task_mqtt_pub(void);

void isr_button(void);
void set_buzzer(int state);

uint8_t get_flame(void);
int16_t get_axis_x(void);
int16_t get_axis_y(void);
int16_t get_axis_z(void);
int16_t get_gyro_x(void);
int16_t get_gyro_y(void);
int16_t get_gyro_z(void);
uint16_t get_gas(void);
int16_t get_temperature(void);
uint16_t get_humidity(void);
uint16_t get_illuminance(void);
uint16_t get_tof(void);

void setup() {

  int ret;
  char c;
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);  // 1MHz, 400kHz, 100kHz

  // Arduino USART setup.
  Serial.begin(115200);

  pinMode(PIN_PT, INPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

#ifndef INL_NOT_USING_SENSOR
  // Initializing the ICM42670P
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM42670P initialization failed: ");
    Serial.println(ret);
    //while(1);
  } else {
    // Accel ODR = 100 Hz and Full Scale Range = 16G
    IMU.startAccel(100,16);
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    IMU.startGyro(100,2000);
    // Wait IMU to start
    delay(100);
    // Plotter axis header
    Serial.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temperature");
  }

  Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }

  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_NO_HEATER);
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }

  Serial.print("Initialization ranging sensor TMF8x01......");
  while(tof.begin() != 0){                                                 //Initialization sensor,sucess return 0, fail return -1
      Serial.println("failed.");
      delay(1000);
  }
  Serial.println("done.");

  Serial.print("Software Version: ");
  Serial.println(tof.getSoftwareVersion());
  Serial.print("Unique ID: ");
  Serial.println(tof.getUniqueID(),HEX);
  Serial.print("Model: ");
  Serial.println(tof.getSensorModel());

  tof.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));                //Set calibration data.


  // @brief Config measurement params to enable measurement. Need to call stopMeasurement to stop ranging action.
  // @param cailbMode: Is an enumerated variable of eCalibModeConfig_t, which is to config measurement cailibration mode.
  // @n     eModeNoCalib  :          Measuring without any calibration data.
  // @n     eModeCalib    :          Measuring with calibration data.
  // @n     eModeCalibAndAlgoState : Measuring with calibration and algorithm state.
  // @param disMode : the ranging mode of TMF8701 sensor.(this mode only TMF8701 support)
  // @n     ePROXIMITY: Raing in PROXIMITY mode,ranging range 0~10cm
  // @n     eDISTANCE: Raing in distance mode,ranging range 10~60cm
  // @n     eCOMBINE:  Raing in PROXIMITY and DISTANCE hybrid mode,ranging range 0~60cm

  tof.startMeasurement(/*cailbMode =*/tof.eModeCalib);                 //Enable measuring with Calibration data.
#endif

#if (ENABLE_BLE_OP == 1) // Enable BLE init
  Serial.println("init_stage_00");
  BLEDevice::init(DEVICE_NAME);
  Serial.println("init_stage_01");
  pServer = BLEDevice::createServer();
  Serial.println("init_stage_02");
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("init_stage_03");

  init_service();
  init_beacon();

  Serial.println("iBeacon + service defined and advertising!");
#endif

  uint8_t mdata[6];
  esp_read_mac(mdata, ESP_MAC_WIFI_STA);
  Serial.printf("Base MAC: %02x, %02x, %02x, %02x, %02x, %02x\r\n", mdata[0], mdata[1], mdata[2], mdata[3], mdata[4], mdata[5]);
  esp_read_mac(mdata, ESP_MAC_BT);
  Serial.printf("Base BT: %02x, %02x, %02x, %02x, %02x, %02x\r\n", mdata[0], mdata[1], mdata[2], mdata[3], mdata[4], mdata[5]);

  int16_t widata;
  widata = -1;
  Serial.printf("widata: %04x\r\n", widata);
  int8_t bidata;
  bidata = -1;
  Serial.printf("bidata: %02x\r\n", bidata);

  attachInterrupt(PIN_BUTTON, isr_button, FALLING);

#if 0
  EEPROM.begin(EEPROM_SIZE);

  eeprom_write_data = 42;
  EEPROM.write(eeprom_addr, eeprom_write_data);
  Serial.println("EEPROM Data Write Done");

  eeprom_read_data = EEPROM.read(eeprom_addr);
  Serial.print("EEPROM Read Data: ");
  Serial.println(eeprom_read_data);

  get_eeprom_data();
  if(gv_ssid[0]) {
    Serial.printf("EEPROM_DATA_SSID: %s\r\n", gv_ssid);
  }
  if(gv_passwd[0]) {
    Serial.printf("EEPROM_DATA_PASSWD: %s\r\n", gv_passwd);
  }
  if(gv_server[0]) {
    Serial.printf("EEPROM_DATA_SERVER: %s\r\n", gv_server);
  }
  if(gv_id[0] | gv_id[1] | gv_id[2] | gv_id[3] | gv_id[4]) {
    Serial.printf("EEPROM_DATA_ID: %02x, %02x, %02x, %02x, %02x\r\n", gv_id[0], gv_id[1], gv_id[2], gv_id[3] , gv_id[4]);
  }
#endif

  uint8_t plen;
  uint8_t rdata[EEPROM_ADDR_SN_NUM_SZ+EEPROM_ADDR_SSID_SZ+EEPROM_ADDR_PASSWD_SZ+EEPROM_ADDR_SERVER_SZ];
  prefs.begin("settings");
  memset((void*) &gv_settings, 0x00, sizeof(gv_settings));
  if(prefs.isKey("settings")) {
    plen = prefs.getBytesLength("settings");
    if(plen){
      prefs.getBytes("settings", rdata, plen);
      memcpy((void*)&gv_settings, rdata, plen);
      Serial.printf("ID: %02x, %02x, %02x, %02x, %02x\r\n", gv_settings.gv_id[0], gv_settings.gv_id[1], 
        gv_settings.gv_id[2], gv_settings.gv_id[3], gv_settings.gv_id[4]);
      Serial.printf("SSID: %s\r\n", gv_settings.gv_ssid);
      Serial.printf("PASSWD: %s\r\n", gv_settings.gv_passwd);
      Serial.printf("Server: %s\r\n", gv_settings.gv_server);
    }
    for(int i = 0; i < 5; i++){
      gv_id[i] = gv_settings.gv_id[i];
    }
  }

  if(isalnum(gv_settings.gv_ssid[0])){
    strcpy(gv_ssid, gv_settings.gv_ssid);
  } else {
    strlcpy(gv_ssid, ssid, strlen(ssid));
  }

  if(isalnum(gv_settings.gv_passwd[0])){
    strcpy(gv_passwd, gv_settings.gv_passwd);
  } else {
    strlcpy(gv_passwd, password, strlen(password));
  }

  if(isalnum(gv_settings.gv_server[0])){
    strcpy(gv_server, gv_settings.gv_server);
  } else {
    strlcpy(gv_server, mqttServer, strlen(mqttServer));
  }

  Serial.printf("ID: %02x, %02x, %02x, %02x, %02x\r\n", gv_id[0], gv_id[1], gv_id[2], gv_id[3], gv_id[4]);
  Serial.printf("SSID: %s\r\n", gv_ssid);
  Serial.printf("PASSWD: %s\r\n", gv_passwd);
  Serial.printf("Server: %s\r\n", gv_server);

#if (ENABLE_MQTT_OP == 1)
  Serial.print("Connecting to WiFi");
  WiFi.begin(gv_ssid, gv_passwd);
  while (WiFi.status() != WL_CONNECTED) {
    if(Serial.available()) {
      c = Serial.read();
      if(c == 27) break;
    }
    delay(200);
    Serial.print(".");
  }
  if(WiFi.status() == WL_CONNECTED){
    wifi_connected = 1;
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt_client.setServer(gv_server, mqtt_port);
  mqtt_client.setCallback(mqtt_callback);

  if(wifi_connected) connectToMQTT();
#endif
}

void loop() {

  Serial.println();
  Serial.println("INL-D01 Main Loop");
  Serial.println("(C) 2023 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_INL_D01_FW);

  char c;
  unsigned long cur_millis = 0;
  unsigned long prev_millis = 0;
  uint8_t sbt_count = 0;

  sys_start_millis = millis();

  while(1){

    prev_millis = millis();
    if(!sbt_count){
      Serial.printf("Task Interval: %d\r\n", prev_millis);
    }

    if(Serial.available()) {
      c = Serial.read();
    }
    if(c == '#'){
      Serial.println("Go to Sub Test!!!");
      sub_test_loop();
      Serial.println("Return to Main Loop!!!");
      c = 0;
    }
#if (ENABLE_BLE_OP == 1) // Enable BLE
    if(!(sbt_count % 4)){
      if (deviceConnected) {
        Serial.printf("*** NOTIFY: %d ***\r\n", value);
        pCharacteristic->setValue(&value, 1);
        pCharacteristic->notify();
        value++;
      }
    }

    if(!(sbt_count % 10)){
      if (!deviceConnected) {
        Serial.printf("*** Custom UUID Byte: %02x\r\n", custom_uuid_byte_0);
        if(sbt_count == 0){
          update_beacon_a();
        } else {
          update_beacon_b();
        }
        custom_uuid_byte_0++;
      }
    }
#endif
    if(sbt_count == 1){
      sub_task_flame();
    } else if(sbt_count == 2){
      sub_task_three_axis();
    } else if(sbt_count == 3){
      sub_task_gas();
    } else if(sbt_count == 5){
      sub_task_temperature();
    } else if(sbt_count == 6){
      sub_task_illuminance();
    } else if(sbt_count == 7){
      sub_task_tof();
    } 
#if (ENABLE_MQTT_OP == 1)
    else if(sbt_count == 9){
      sub_task_mqtt_pub();
    }
    if(wifi_connected){
      if (!mqtt_client.connected()) {
        connectToMQTT();
      }
      mqtt_client.loop();
    }
#endif

    // wait until the interval
    while(1){
      if(ug_button_isr_detected){
        Serial.printf("UG_Button_Detected: %d\r\n", ug_button_status);
        if(ug_button_status == UG_BUTTON_PRESSED) {
          set_buzzer(STATE_BUZZER_ON);
        } else if(ug_button_status == UG_BUTTON_RELEASED) {
          set_buzzer(STATE_BUZZER_OFF);
        }
        ug_button_isr_detected = 0;
      }
      cur_millis = millis();
      if((cur_millis - prev_millis) < SINGLE_TASK_PERIOD){
        delay(10);
      } else {
        break;
      }
    }

    if(++sbt_count >= 20){
      sbt_count = 0;
    }

  }
  Serial.println("loop exit!");
}

void sub_test_loop(void){

  Serial.println();
  Serial.println("INL-D01 Sub-Testing");
  Serial.println("(C) 2023 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_INL_D01_FW);

  char c;
  while(c != 'x') {
    Serial.printf("Input Command: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)){
          break;
        }
      }
      delay(100);
    }
    Serial.printf("%c", c);
    Serial.println();

    switch(c) {
      case 'a': 
        sub_test_a();
        break;
      case 'b':
        sub_test_b();
        break;
      case 'c':
        sub_test_c();
        break;
      case 'd':
        sub_test_d();
        break;
      case 'e':
        sub_test_e();
        break;
      case 'g':
        sub_test_g();
        break;
      case 'h':
        sub_test_h();
        break;
      case 's':
        sub_test_s();
        break;
      case 't':
        sub_test_t();
        break;
      case 'u':
        sub_test_u();
        break;
      case 'v':
        sub_test_v();
        break;
      case 'w':
        sub_test_w();
        break;
      default:
        break;
    }
  }
  Serial.println("loop exit!");
}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(addr, (uint8_t)dlen);

  if (Wire.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = Wire.read();
    }
  }

  return ret;
}

int i2c_read_wo(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(addr, (size_t)dlen, (bool) false);

  if (Wire.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = Wire.read();
    }
  }

  return ret;
}

int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  for(int i = 0; i < dlen; i++) {
    Wire.write(pdata[i]);
  }
  Wire.endTransmission();

  return ret;
}

int get_char(uint8_t in_byte, char* pout_bytes){

  uint8_t ret = 0;
  uint8_t tdata = 0;

  tdata = (in_byte >> 4)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    pout_bytes[0] = tdata + '0';
  } else {
    pout_bytes[0] = (tdata%10) + 'A';
  }
  tdata = (in_byte)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    pout_bytes[1] = tdata + '0';
  } else {
    pout_bytes[1] = (tdata%10) + 'A';
  }

  return ret;
}

void init_service() {

  BLEAdvertising* pAdvertising;
  pAdvertising = pServer->getAdvertising();
  pAdvertising->stop();

  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pAdvertising->addServiceUUID(BLEUUID(SERVICE_UUID));

  // Start the service
  pService->start();

  pAdvertising->start();
}

void init_beacon() {

  BLEAdvertising* pAdvertising;
  pAdvertising = pServer->getAdvertising();
  pAdvertising->stop();
  // iBeacon
  BLEBeacon myBeacon;
  myBeacon.setManufacturerId(0x4c00);
  myBeacon.setMajor(5);
  myBeacon.setMinor(88);
  myBeacon.setSignalPower(0xc5);
  myBeacon.setProximityUUID(BLEUUID(BEACON_UUID_REV));

  BLEAdvertisementData advertisementData;
  advertisementData.setFlags(0x1A);
  advertisementData.setManufacturerData(myBeacon.getData());
  pAdvertising->setAdvertisementData(advertisementData);

  pAdvertising->start();
}

void update_beacon() {

  BLEAdvertising* pAdvertising;
  pAdvertising = pServer->getAdvertising();
  pAdvertising->stop();
  // iBeacon
  BLEBeacon myBeacon;
  myBeacon.setManufacturerId(0x4c00);
  //myBeacon.setMajor(custom_uuid_byte_0);
  //myBeacon.setMinor(0x00ff - custom_uuid_byte_0);
  //Serial.printf("Update Major: %d, Minor: %d\r\n", custom_uuid_byte_0, 0x00ff - custom_uuid_byte_0);
  myBeacon.setSignalPower(0xc5);
#if 0
  String myUUID(custom_beacon_uuid);
  uint8_t tdata = 0;
  tdata = (custom_uuid_byte_0 >> 4)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    myUUID.setCharAt(0, tdata + '0'); 
  } else {
    myUUID.setCharAt(0, (tdata%10) + 'A');
  }
  tdata = (custom_uuid_byte_0)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    myUUID.setCharAt(1, tdata + '0'); 
  } else {
    myUUID.setCharAt(1, (tdata%10) + 'A');
  }
  Serial.printf("myUUID: %c, %c\r\n", myUUID.charAt(0), myUUID.charAt(1));
  Serial.printf("myUUID: %s\r\n", myUUID.c_str());
  myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
#elif 0
  String myUUID(custom_beacon_uuid);
  myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
#elif 1
  String myUUID(custom_beacon_uuid);
  String myUUID_R(custom_beacon_uuid);
  uint8_t tbdata = 0;
  int8_t tbidata = 0;
  uint16_t twdata = 0;
  int16_t twidata = 0;
  char tcdata[3] = {0,};
  uint8_t tadata[20] = {0,};

  // Identify char
  tbdata = 'D'; 
  tadata[0] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(0, tcdata[0]);
  myUUID.setCharAt(1, tcdata[1]);
  myUUID_R.setCharAt(34, tcdata[0]);
  myUUID_R.setCharAt(35, tcdata[1]);
  tbdata = 'S';
  tadata[1] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(2, tcdata[0]);
  myUUID.setCharAt(3, tcdata[1]);
  myUUID_R.setCharAt(32, tcdata[0]);
  myUUID_R.setCharAt(33, tcdata[1]);
  tbdata = 'I';
  tadata[2] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(4, tcdata[0]);
  myUUID.setCharAt(5, tcdata[1]);
  myUUID_R.setCharAt(30, tcdata[0]);
  myUUID_R.setCharAt(31, tcdata[1]);

  // Flame
  tbdata = get_flame(); //0;
  tadata[3] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(6, tcdata[0]);
  myUUID.setCharAt(7, tcdata[1]);
  myUUID_R.setCharAt(28, tcdata[0]);
  myUUID_R.setCharAt(29, tcdata[1]);

  // Acc-X
  twidata = get_axis_x(); //120;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[4] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(9, tcdata[0]);
  myUUID.setCharAt(10, tcdata[1]);
  myUUID_R.setCharAt(26, tcdata[0]);
  myUUID_R.setCharAt(27, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[5] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(11, tcdata[0]);
  myUUID.setCharAt(12, tcdata[1]);
  myUUID_R.setCharAt(24, tcdata[0]);
  myUUID_R.setCharAt(25, tcdata[1]);

  // Acc-Y
  twidata = get_axis_y(); //10;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[6] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(14, tcdata[0]);
  myUUID.setCharAt(15, tcdata[1]);
  myUUID_R.setCharAt(21, tcdata[0]);
  myUUID_R.setCharAt(22, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[7] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(16, tcdata[0]);
  myUUID.setCharAt(17, tcdata[1]);
  myUUID_R.setCharAt(19, tcdata[0]);
  myUUID_R.setCharAt(20, tcdata[1]);

  // Acc-Z
  twidata = get_axis_z(); //-10;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[8] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(19, tcdata[0]);
  myUUID.setCharAt(20, tcdata[1]);
  myUUID_R.setCharAt(16, tcdata[0]);
  myUUID_R.setCharAt(17, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[9] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(21, tcdata[0]);
  myUUID.setCharAt(22, tcdata[1]);
  myUUID_R.setCharAt(14, tcdata[0]);
  myUUID_R.setCharAt(15, tcdata[1]);

  // MQ-2
  twdata = get_gas(); //320;
  tbdata = ((twdata >> 8)&0x00ff);
  tadata[10] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(24, tcdata[0]);
  myUUID.setCharAt(25, tcdata[1]);
  myUUID_R.setCharAt(11, tcdata[0]);
  myUUID_R.setCharAt(12, tcdata[1]);
  tbdata = ((twdata >> 0)&0x00ff);
  tadata[11] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(26, tcdata[0]);
  myUUID.setCharAt(27, tcdata[1]);
  myUUID_R.setCharAt(9, tcdata[0]);
  myUUID_R.setCharAt(10, tcdata[1]);

  // Temperature
  tbidata = get_temperature(); //27;
  tadata[12] = tbidata;
  get_char(tbidata, tcdata);
  myUUID.setCharAt(28, tcdata[0]);
  myUUID.setCharAt(29, tcdata[1]);
  myUUID_R.setCharAt(6, tcdata[0]);
  myUUID_R.setCharAt(7, tcdata[1]);

  // Humidity
  tbdata = get_humidity(); //25;
  tadata[13] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(30, tcdata[0]);
  myUUID.setCharAt(31, tcdata[1]);
  myUUID_R.setCharAt(4, tcdata[0]);
  myUUID_R.setCharAt(5, tcdata[1]);

  // LUX
  twdata = get_illuminance(); //1200;
  tbdata = ((twdata >> 8)&0x00ff);
  tadata[14] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(32, tcdata[0]);
  myUUID.setCharAt(33, tcdata[1]);
  myUUID_R.setCharAt(2, tcdata[0]);
  myUUID_R.setCharAt(3, tcdata[1]);
  tbdata = ((twdata >> 0)&0x00ff);
  tadata[15] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(34, tcdata[0]);
  myUUID.setCharAt(35, tcdata[1]);
  myUUID_R.setCharAt(0, tcdata[0]);
  myUUID_R.setCharAt(1, tcdata[1]);

  // TOF
  twdata = get_tof(); //1500;
  tbdata = ((twdata >> 8)&0x00ff);
  tadata[16] = tbdata;
  tbdata = ((twdata >> 0)&0x00ff);
  tadata[17] = tbdata;
  myBeacon.setMajor(twdata);

  // CRC
  setCRC(tadata, 20);
  twdata = tadata[18]*0x100;
  twdata += tadata[19];
  //twdata = 0xabcd;
  myBeacon.setMinor(twdata);
  Serial.printf("Major: 0x%04x, Minor: 0x%04x\r\n", get_tof(), twdata);
  Serial.printf("myUUID: %s\r\n", myUUID.c_str());
  //myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
  myBeacon.setProximityUUID(BLEUUID(myUUID_R.c_str()));
#else
  myBeacon.setProximityUUID(BLEUUID(BEACON_UUID_REV));
#endif

  BLEAdvertisementData advertisementData;
  advertisementData.setFlags(0x1A);
  advertisementData.setManufacturerData(myBeacon.getData());
  pAdvertising->setAdvertisementData(advertisementData);

  pAdvertising->start();
}

void update_beacon_a() {

  BLEAdvertising* pAdvertising;
  pAdvertising = pServer->getAdvertising();
  pAdvertising->stop();
  // iBeacon
  BLEBeacon myBeacon;
  myBeacon.setManufacturerId(0x4c00);
  //myBeacon.setMajor(custom_uuid_byte_0);
  //myBeacon.setMinor(0x00ff - custom_uuid_byte_0);
  //Serial.printf("Update Major: %d, Minor: %d\r\n", custom_uuid_byte_0, 0x00ff - custom_uuid_byte_0);
  myBeacon.setSignalPower(0xc5);
#if 0
  String myUUID(custom_beacon_uuid);
  uint8_t tdata = 0;
  tdata = (custom_uuid_byte_0 >> 4)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    myUUID.setCharAt(0, tdata + '0'); 
  } else {
    myUUID.setCharAt(0, (tdata%10) + 'A');
  }
  tdata = (custom_uuid_byte_0)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    myUUID.setCharAt(1, tdata + '0'); 
  } else {
    myUUID.setCharAt(1, (tdata%10) + 'A');
  }
  Serial.printf("myUUID: %c, %c\r\n", myUUID.charAt(0), myUUID.charAt(1));
  Serial.printf("myUUID: %s\r\n", myUUID.c_str());
  myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
#elif 0
  String myUUID(custom_beacon_uuid);
  myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
#elif 1
  String myUUID(custom_beacon_uuid);
  String myUUID_R(custom_beacon_uuid);
  uint8_t tbdata = 0;
  int8_t tbidata = 0;
  uint16_t twdata = 0;
  int16_t twidata = 0;
  char tcdata[3] = {0,};
  uint8_t tadata[20] = {0,};

  // Identify char
  tbdata = 'D'; 
  tadata[0] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(0, tcdata[0]);
  myUUID.setCharAt(1, tcdata[1]);
  myUUID_R.setCharAt(34, tcdata[0]);
  myUUID_R.setCharAt(35, tcdata[1]);
  tbdata = 'S';
  tadata[1] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(2, tcdata[0]);
  myUUID.setCharAt(3, tcdata[1]);
  myUUID_R.setCharAt(32, tcdata[0]);
  myUUID_R.setCharAt(33, tcdata[1]);
  tbdata = 'I';
  tadata[2] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(4, tcdata[0]);
  myUUID.setCharAt(5, tcdata[1]);
  myUUID_R.setCharAt(30, tcdata[0]);
  myUUID_R.setCharAt(31, tcdata[1]);

  // N.A.
  tbdata = 0;
  tadata[3] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(6, tcdata[0]);
  myUUID.setCharAt(7, tcdata[1]);
  myUUID_R.setCharAt(28, tcdata[0]);
  myUUID_R.setCharAt(29, tcdata[1]);

  // Acc-X
  twidata = get_axis_x(); //120;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[4] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(9, tcdata[0]);
  myUUID.setCharAt(10, tcdata[1]);
  myUUID_R.setCharAt(26, tcdata[0]);
  myUUID_R.setCharAt(27, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[5] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(11, tcdata[0]);
  myUUID.setCharAt(12, tcdata[1]);
  myUUID_R.setCharAt(24, tcdata[0]);
  myUUID_R.setCharAt(25, tcdata[1]);

  // Acc-Y
  twidata = get_axis_y(); //10;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[6] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(14, tcdata[0]);
  myUUID.setCharAt(15, tcdata[1]);
  myUUID_R.setCharAt(21, tcdata[0]);
  myUUID_R.setCharAt(22, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[7] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(16, tcdata[0]);
  myUUID.setCharAt(17, tcdata[1]);
  myUUID_R.setCharAt(19, tcdata[0]);
  myUUID_R.setCharAt(20, tcdata[1]);

  // Acc-Z
  twidata = get_axis_z(); //-10;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[8] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(19, tcdata[0]);
  myUUID.setCharAt(20, tcdata[1]);
  myUUID_R.setCharAt(16, tcdata[0]);
  myUUID_R.setCharAt(17, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[9] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(21, tcdata[0]);
  myUUID.setCharAt(22, tcdata[1]);
  myUUID_R.setCharAt(14, tcdata[0]);
  myUUID_R.setCharAt(15, tcdata[1]);

  // Gyro-X
  twidata = get_gyro_x(); //5;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[10] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(24, tcdata[0]);
  myUUID.setCharAt(25, tcdata[1]);
  myUUID_R.setCharAt(11, tcdata[0]);
  myUUID_R.setCharAt(12, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[11] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(26, tcdata[0]);
  myUUID.setCharAt(27, tcdata[1]);
  myUUID_R.setCharAt(9, tcdata[0]);
  myUUID_R.setCharAt(10, tcdata[1]);

  // Gyro-Y
  twidata = get_gyro_y(); //-3;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[12] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(28, tcdata[0]);
  myUUID.setCharAt(29, tcdata[1]);
  myUUID_R.setCharAt(6, tcdata[0]);
  myUUID_R.setCharAt(7, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[13] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(30, tcdata[0]);
  myUUID.setCharAt(31, tcdata[1]);
  myUUID_R.setCharAt(4, tcdata[0]);
  myUUID_R.setCharAt(5, tcdata[1]);

  // Gyro-Z
  twidata = get_gyro_z(); //-7;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[14] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(32, tcdata[0]);
  myUUID.setCharAt(33, tcdata[1]);
  myUUID_R.setCharAt(2, tcdata[0]);
  myUUID_R.setCharAt(3, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[15] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(34, tcdata[0]);
  myUUID.setCharAt(35, tcdata[1]);
  myUUID_R.setCharAt(0, tcdata[0]);
  myUUID_R.setCharAt(1, tcdata[1]);

  // Type
  tbdata = 'A';
  twdata = tbdata*0x100 + 'A';
  tadata[16] = tbdata;
  tadata[17] = tbdata;
  myBeacon.setMajor(twdata);

  // CRC
  setCRC(tadata, 20);
  twdata = tadata[18]*0x100;
  twdata += tadata[19];
  //twdata = 0xabcd;
  myBeacon.setMinor(twdata);
  Serial.printf("Major: 0x%04x, Minor: 0x%04x\r\n", myBeacon.getMajor(), twdata);
  Serial.printf("myUUID: %s\r\n", myUUID.c_str());
  //myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
  myBeacon.setProximityUUID(BLEUUID(myUUID_R.c_str()));
#else
  myBeacon.setProximityUUID(BLEUUID(BEACON_UUID_REV));
#endif

  BLEAdvertisementData advertisementData;
  advertisementData.setFlags(0x1A);
  advertisementData.setManufacturerData(myBeacon.getData());
  pAdvertising->setAdvertisementData(advertisementData);

  pAdvertising->start();
}

void update_beacon_b() {

  BLEAdvertising* pAdvertising;
  pAdvertising = pServer->getAdvertising();
  pAdvertising->stop();
  // iBeacon
  BLEBeacon myBeacon;
  myBeacon.setManufacturerId(0x4c00);
  //myBeacon.setMajor(custom_uuid_byte_0);
  //myBeacon.setMinor(0x00ff - custom_uuid_byte_0);
  //Serial.printf("Update Major: %d, Minor: %d\r\n", custom_uuid_byte_0, 0x00ff - custom_uuid_byte_0);
  myBeacon.setSignalPower(0xc5);
#if 0
  String myUUID(custom_beacon_uuid);
  uint8_t tdata = 0;
  tdata = (custom_uuid_byte_0 >> 4)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    myUUID.setCharAt(0, tdata + '0'); 
  } else {
    myUUID.setCharAt(0, (tdata%10) + 'A');
  }
  tdata = (custom_uuid_byte_0)&0x0f;
  if((tdata >= 0) && (tdata <= 9)){
    myUUID.setCharAt(1, tdata + '0'); 
  } else {
    myUUID.setCharAt(1, (tdata%10) + 'A');
  }
  Serial.printf("myUUID: %c, %c\r\n", myUUID.charAt(0), myUUID.charAt(1));
  Serial.printf("myUUID: %s\r\n", myUUID.c_str());
  myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
#elif 0
  String myUUID(custom_beacon_uuid);
  myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
#elif 1
  String myUUID(custom_beacon_uuid);
  String myUUID_R(custom_beacon_uuid);
  uint8_t tbdata = 0;
  int8_t tbidata = 0;
  uint16_t twdata = 0;
  int16_t twidata = 0;
  char tcdata[3] = {0,};
  uint8_t tadata[20] = {0,};

  // Identify char
  tbdata = 'D'; 
  tadata[0] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(0, tcdata[0]);
  myUUID.setCharAt(1, tcdata[1]);
  myUUID_R.setCharAt(34, tcdata[0]);
  myUUID_R.setCharAt(35, tcdata[1]);
  tbdata = 'S';
  tadata[1] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(2, tcdata[0]);
  myUUID.setCharAt(3, tcdata[1]);
  myUUID_R.setCharAt(32, tcdata[0]);
  myUUID_R.setCharAt(33, tcdata[1]);
  tbdata = 'I';
  tadata[2] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(4, tcdata[0]);
  myUUID.setCharAt(5, tcdata[1]);
  myUUID_R.setCharAt(30, tcdata[0]);
  myUUID_R.setCharAt(31, tcdata[1]);

  // N.A.
  tbdata = 0;
  tadata[3] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(6, tcdata[0]);
  myUUID.setCharAt(7, tcdata[1]);
  myUUID_R.setCharAt(28, tcdata[0]);
  myUUID_R.setCharAt(29, tcdata[1]);

  // Temperature
  twidata = get_temperature(); //27;
  tbdata = ((twidata >> 8)&0x00ff);
  tadata[4] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(9, tcdata[0]);
  myUUID.setCharAt(10, tcdata[1]);
  myUUID_R.setCharAt(26, tcdata[0]);
  myUUID_R.setCharAt(27, tcdata[1]);
  tbdata = ((twidata >> 0)&0x00ff);
  tadata[5] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(11, tcdata[0]);
  myUUID.setCharAt(12, tcdata[1]);
  myUUID_R.setCharAt(24, tcdata[0]);
  myUUID_R.setCharAt(25, tcdata[1]);

  // Humidity
  twdata = get_humidity(); //25;
  tbdata = ((twdata >> 8)&0x00ff);
  tadata[6] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(14, tcdata[0]);
  myUUID.setCharAt(15, tcdata[1]);
  myUUID_R.setCharAt(21, tcdata[0]);
  myUUID_R.setCharAt(22, tcdata[1]);
  tbdata = ((twdata >> 0)&0x00ff);
  tadata[7] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(16, tcdata[0]);
  myUUID.setCharAt(17, tcdata[1]);
  myUUID_R.setCharAt(19, tcdata[0]);
  myUUID_R.setCharAt(20, tcdata[1]);

  // LUX
  twdata = get_illuminance(); //1200;
  tbdata = ((twdata >> 8)&0x00ff);
  tadata[8] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(19, tcdata[0]);
  myUUID.setCharAt(20, tcdata[1]);
  myUUID_R.setCharAt(16, tcdata[0]);
  myUUID_R.setCharAt(17, tcdata[1]);
  tbdata = ((twdata >> 0)&0x00ff);
  tadata[9] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(21, tcdata[0]);
  myUUID.setCharAt(22, tcdata[1]);
  myUUID_R.setCharAt(14, tcdata[0]);
  myUUID_R.setCharAt(15, tcdata[1]);

  // MQ-2
  twdata = get_gas(); //320;
  tbdata = ((twdata >> 8)&0x00ff);
  tadata[10] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(24, tcdata[0]);
  myUUID.setCharAt(25, tcdata[1]);
  myUUID_R.setCharAt(11, tcdata[0]);
  myUUID_R.setCharAt(12, tcdata[1]);
  tbdata = ((twdata >> 0)&0x00ff);
  tadata[11] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(26, tcdata[0]);
  myUUID.setCharAt(27, tcdata[1]);
  myUUID_R.setCharAt(9, tcdata[0]);
  myUUID_R.setCharAt(10, tcdata[1]);

  // TOF
  twdata = get_tof(); //1500;
  tbdata = ((twdata >> 8)&0x00ff);
  tadata[12] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(28, tcdata[0]);
  myUUID.setCharAt(29, tcdata[1]);
  myUUID_R.setCharAt(6, tcdata[0]);
  myUUID_R.setCharAt(7, tcdata[1]);
  tbdata = ((twdata >> 0)&0x00ff);
  tadata[13] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(30, tcdata[0]);
  myUUID.setCharAt(31, tcdata[1]);
  myUUID_R.setCharAt(4, tcdata[0]);
  myUUID_R.setCharAt(5, tcdata[1]);

  // Flame
  tbdata = get_flame(); //0;
  tadata[14] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(32, tcdata[0]);
  myUUID.setCharAt(33, tcdata[1]);
  myUUID_R.setCharAt(2, tcdata[0]);
  myUUID_R.setCharAt(3, tcdata[1]);

  // N.A.
  tbdata = 0;
  tadata[15] = tbdata;
  get_char(tbdata, tcdata);
  myUUID.setCharAt(34, tcdata[0]);
  myUUID.setCharAt(35, tcdata[1]);
  myUUID_R.setCharAt(0, tcdata[0]);
  myUUID_R.setCharAt(1, tcdata[1]);

  // Type
  tbdata = 'B';
  twdata = tbdata*0x100 + 'B';
  tadata[16] = tbdata;
  tadata[17] = tbdata;
  myBeacon.setMajor(twdata);

  // CRC
  setCRC(tadata, 20);
  twdata = tadata[18]*0x100;
  twdata += tadata[19];
  //twdata = 0xabcd;
  myBeacon.setMinor(twdata);
  Serial.printf("Major: 0x%04x, Minor: 0x%04x\r\n", myBeacon.getMajor(), twdata);
  Serial.printf("myUUID: %s\r\n", myUUID.c_str());
  //myBeacon.setProximityUUID(BLEUUID(myUUID.c_str()));
  myBeacon.setProximityUUID(BLEUUID(myUUID_R.c_str()));
#else
  myBeacon.setProximityUUID(BLEUUID(BEACON_UUID_REV));
#endif

  BLEAdvertisementData advertisementData;
  advertisementData.setFlags(0x1A);
  advertisementData.setManufacturerData(myBeacon.getData());
  pAdvertising->setAdvertisementData(advertisementData);

  pAdvertising->start();
}

bool checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}

void setCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}

uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value
    uint8_t idx = 0;

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}

void isr_button(void) {

  if(millis() > (last_isr_millis + UG_BUTTON_DEBOUNCE)){

    if(ug_button_status == UG_BUTTON_RELEASED){
      ug_button_status = UG_BUTTON_PRESSED;
    } else {
      ug_button_status = UG_BUTTON_RELEASED;
    }
    ug_button_isr_detected = 1;
    last_isr_millis = millis();
  }
}

void set_buzzer(int state){
  
  if(state == STATE_BUZZER_OFF){

    digitalWrite(PIN_BUZZER, LOW);

  } else if(state == STATE_BUZZER_ON){

    digitalWrite(PIN_BUZZER, HIGH);

  }
}

void crc32_update(const uint32_t data)
{
    uint32_t tbl_idx = 0;
    tbl_idx = crc32_state ^ (data >> (0 * 4));
    crc32_state = crc32_table[(tbl_idx & 0x0f)] ^ (crc32_state >> 4);
    tbl_idx = crc32_state ^ (data >> (1 * 4));
    crc32_state = crc32_table[(tbl_idx & 0x0f)] ^ (crc32_state >> 4);
}

void connectToMQTT(void) {

  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    long r = random(1000);
    sprintf(mqtt_clientId, "clientId-%ld", r);
    if (mqtt_client.connect(mqtt_clientId)) {
      Serial.printf(" connected[%d]\r\n", r);
      char topic[100];
      snprintf(topic, sizeof(topic), "INLD01/%04X/%02X%02X%02X%02X%02X/%02X", gv_address, 
        gv_id[0], gv_id[1], gv_id[2], gv_id[3], gv_id[4], gv_location);
      mqtt_client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqtt_callback(char* topic, byte* message, unsigned int length) {

  String stMessage;
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    stMessage += (char)message[i];
  }
  Serial.println();
  if (String(topic) == "INLD01/0001/0000000000/01") {
    
    Serial.print("Changing output to ");
    if(stMessage == "on"){
      Serial.println("on");
      digitalWrite(PIN_BUZZER, HIGH);
    }
    else if(stMessage == "off"){
      Serial.println("off");
      digitalWrite(PIN_BUZZER, LOW);
    }
  }
}

void get_eeprom_data(void) {

  uint8_t rdata;

  for(int i = 0; i < 5; i++){
    gv_id[i] = EEPROM.readByte(EEPROM_ADDR_SN_NUM+i);
  }

  rdata = EEPROM.readByte(EEPROM_ADDR_SSID);
  if(isalnum(rdata)){
    EEPROM.readBytes(EEPROM_ADDR_SSID, gv_ssid, sizeof(gv_ssid));
  }

  rdata = EEPROM.readByte(EEPROM_ADDR_PASSWD);
  if(isalnum(rdata)){
    EEPROM.readBytes(EEPROM_ADDR_PASSWD, gv_passwd, sizeof(gv_passwd));
  }

  rdata = EEPROM.readByte(EEPROM_ADDR_SERVER);
  if(isalnum(rdata)){
    EEPROM.readBytes(EEPROM_ADDR_SERVER, gv_server, sizeof(gv_server));
  }
}

void set_eeprom_data_sn_num(void) {

  EEPROM.writeBytes(EEPROM_ADDR_SN_NUM, gv_id, sizeof(gv_id));
}

void set_eeprom_data_ssid(void) {

  EEPROM.writeBytes(EEPROM_ADDR_SSID, gv_ssid, sizeof(gv_ssid));
}

void set_eeprom_data_passwd(void) {

  EEPROM.writeBytes(EEPROM_ADDR_PASSWD, gv_passwd, sizeof(gv_passwd));
}

void set_eeprom_data_server(void) {

  EEPROM.writeBytes(EEPROM_ADDR_SERVER, gv_server, sizeof(gv_server));
}

void sub_task_flame(void) {

#ifdef INL_NOT_USING_SENSOR
  return;
#endif

  gvi_flame = gv_flame = digitalRead(PIN_PT);
  Serial.printf("Flame Sensor: %d\r\n", gv_flame);

}

void sub_task_three_axis(void) {

  inv_imu_sensor_event_t imu_event;

#ifdef INL_NOT_USING_SENSOR
  return;
#endif

  IMU.getDataFromRegisters(&imu_event);

  gv_axis_x = gvf_acc_x = imu_event.accel[0];
  gv_axis_y = gvf_acc_y = imu_event.accel[1];
  gv_axis_z = gvf_acc_z = imu_event.accel[2];
  gv_gyro_x = gvf_gyro_x = imu_event.gyro[0];
  gv_gyro_y = gvf_gyro_y = imu_event.gyro[1];
  gv_gyro_z = gvf_gyro_z = imu_event.gyro[2];
  Serial.printf("3 Axis Sensor: %d, %d, %d. Gyro: %d, %d, %d\r\n", 
    gv_axis_x, gv_axis_y, gv_axis_z, gv_gyro_x, gv_gyro_y, gv_gyro_z);

}

void sub_task_gas(void) {

#ifdef INL_NOT_USING_SENSOR
  return;
#endif

  gvi_gas = gv_gas = analogRead(PIN_MQ2_AO);

  if(!mq2_warm_up_end){
    if(millis() > sys_start_millis + MQ2_WARM_UP_TIME){
      mq2_warm_up_end = 1;
      Serial.printf("MQ-2 Warm-up end: %d\r\n", gv_gas);
    }
  }

  Serial.printf("MQ-2 Gas Sensor: %d\r\n", gv_gas);
  if(!mq2_warm_up_end){
    gv_gas = 0;
  }

}

void sub_task_temperature(void) {

  sensors_event_t humidity, temp;

#ifdef INL_NOT_USING_SENSOR
  return;
#endif

  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  gv_temperature = (int8_t) (gvf_temperature = temp.temperature);
  gv_humidity = (uint8_t) (gvf_humidity = humidity.relative_humidity);
  gv_temperature = temp.temperature*100.;
  gv_humidity = humidity.relative_humidity*100.;

  Serial.printf("Temperature: %f(%d), Humidity: %f(%d)\r\n", temp.temperature, gv_temperature, 
    humidity.relative_humidity, gv_humidity);

}

void sub_task_illuminance(void) {

  uint8_t data[3] = {0,};
  uint16_t out_als = 0;
  uint16_t out_w = 0;
  int val = 0;
  static uint8_t mes_start = 1;

#ifdef INL_NOT_USING_SENSOR
  return;
#endif

  if(mes_start){
    mes_start = 0;
    i2c_write(0x10, 0x00, data, 2);  // start measurement
  }

  // white
  i2c_read_wo(0x10, 0x04, data, 2);
  out_w = 0x100*data[1] + data[0];

  // als 
  i2c_read_wo(0x10, 0x05, data, 2);
  out_als = 0x100*data[1] + data[0];

  val = (int) (((float) out_als)*0.27264);
  gv_illuminance = (uint16_t) val;

  Serial.printf("Illuminance Sensor: %d\r\n", gv_illuminance);

}

void sub_task_tof(void) {

#ifdef INL_NOT_USING_SENSOR
  return;
#endif

  if (tof.isDataReady()) {
    gvf_tof = gv_tof = tof.getDistance_mm();
  }
  Serial.printf("TOF Sensor: %d\r\n", gv_tof);
}

void sub_task_mqtt_pub(void) {

  char topic[100];
  snprintf(topic, sizeof(topic), "INLD01/%04X/%02X%02X%02X%02X%02X/%02X", gv_address, 
    gv_id[0], gv_id[1], gv_id[2], gv_id[3], gv_id[4], gv_location);

#ifdef INL_NOT_USING_SENSOR
  gvf_acc_x = 2000.;
  gvf_acc_y = 100.;
  gvf_acc_z = -100.;
  gvf_gyro_x = 10.;
  gvf_gyro_y = 20.;
  gvf_gyro_z = 30.;
  gvf_temperature = 27.;
  gvf_humidity = 12.;
  gvi_flame = 1.;
  gvi_gas = 34;
  gvf_tof = 567.;
#endif

  DynamicJsonDocument doc(250);

  doc["ax"] = gvf_acc_x;
  doc["ay"] = gvf_acc_y;
  doc["az"] = gvf_acc_z;
  doc["gx"] = gvf_gyro_x;
  doc["gy"] = gvf_gyro_y;
  doc["gz"] = gvf_gyro_z;
  doc["mq2"] = gvi_gas;
  doc["tem"] = gvf_temperature;
  doc["hum"] = gvf_humidity;
  doc["light"] = gv_illuminance;
  doc["flame"] = gvi_flame;
  doc["tof"] = gvf_tof;

  String jsonStringWithCRC;
  serializeJson(doc, jsonStringWithCRC);

  for (size_t i = 0; i < jsonStringWithCRC.length(); i++) {
    crc32_update(jsonStringWithCRC[i]);
  }
  crc32_value = ~crc32_state; 

  doc["crc"] = crc32_value;

  jsonStringWithCRC = "";
  serializeJson(doc, jsonStringWithCRC);

  if (mqtt_client.publish(topic, jsonStringWithCRC.c_str())) {
    Serial.println("Message sent successfully.");
  } else {
    Serial.println("Error sending message.");
  }

}

uint8_t get_flame(void) {

  uint8_t ret;

  if(gv_flame){
    ret = STATE_FLAME_NONE;
  } else{
    ret = STATE_FLAME_DETECTED;
  }
#if (BLE_FORMAT_TEST == 1)
  return 0;
#else
  return ret;
#endif
}

int16_t get_axis_x(void) {

#if (BLE_FORMAT_TEST == 1)
  return 120;
#else
  return gv_axis_x;
#endif
}

int16_t get_axis_y(void) {

#if (BLE_FORMAT_TEST == 1)
  return 10;
#else
  return gv_axis_y;
#endif
}

int16_t get_axis_z(void) {

#if (BLE_FORMAT_TEST == 1)
  return -10;
#else
  return gv_axis_z;
#endif
}

int16_t get_gyro_x(void) {

#if (BLE_FORMAT_TEST == 1)
  return 5;
#else
  return gv_gyro_x;
#endif
}

int16_t get_gyro_y(void) {

#if (BLE_FORMAT_TEST == 1)
  return -3;
#else
  return gv_gyro_y;
#endif
}

int16_t get_gyro_z(void) {

#if (BLE_FORMAT_TEST == 1)
  return -7;
#else
  return gv_gyro_z;
#endif
}

uint16_t get_gas(void) {

#if (BLE_FORMAT_TEST == 1)
  return 320;
#else
  return gv_gas;
#endif
}

int16_t get_temperature(void) {

#if (BLE_FORMAT_TEST == 1)
  return 27;
#else
  return gv_temperature;
#endif
}

uint16_t get_humidity(void) {

#if (BLE_FORMAT_TEST == 1)
  return 25;
#else
  return gv_humidity;
#endif
}

uint16_t get_illuminance(void) {

#if (BLE_FORMAT_TEST == 1)
  return 1200;
#else
  return gv_illuminance;
#endif
}

uint16_t get_tof(void) {

#if (BLE_FORMAT_TEST == 1)
  return 1500;
#else
  return gv_tof;
#endif
}

void sub_test_a(void) {

  uint8_t address = 0;
  uint8_t reg = 0;
  uint8_t dlen = 0;
  uint8_t out[2] = {0,};
  char c;
  Serial.println("Sub-test A - Read I2C");

  Serial.print("Input I2C Address: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    address = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'f')) {
    address = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    address |= (c - '0');
  } else if ((c >= 'a') && (c <= 'f')) {
    address |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Serial.print("Input Register Address: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'f')) {
    reg = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg |= (c - '0');
  } else if ((c >= 'a') && (c <= 'f')) {
    reg |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  if(address == 0x10){
    dlen = 2;
  } else {
    dlen = 1;
  }

  Wire.beginTransmission(address);
  Wire.write(reg);
  if(address == 0x10){
    Wire.endTransmission(false);
  } else {
    Wire.endTransmission();
  }
  
  if(address == 0x10) {
      Wire.requestFrom(address, (size_t)dlen, (bool) false);
  } else {
    Wire.requestFrom(address, (uint8_t)dlen);
  }
  if (Wire.available()) {
    out[0] = Wire.read();
    if(dlen > 1){
      out[1] = Wire.read();
    }
  }
  Serial.print("I2C Read: address: "); Serial.print(address, HEX);
  Serial.print(", register: "); Serial.print(reg, HEX);
  Serial.print(", value: "); 
  if(address == 0x10) {
    Serial.print(out[0], HEX); Serial.print(", "); Serial.println(out[1], HEX);
  } else {
    Serial.println(out[0], HEX);
  }

}

void sub_test_b(void) {

  uint8_t address = 0;
  uint8_t reg = 0;
  uint8_t idx = 0;
  uint8_t val[2] = {0,};
  char c;
  Serial.println("Sub-test B - Write I2C");

  Serial.print("Input I2C Address: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    address = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'f')) {
    address = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    address |= (c - '0');
  } else if ((c >= 'a') && (c <= 'f')) {
    address |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Serial.print("Input Register Address: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'f')) {
    reg = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg |= (c - '0');
  } else if ((c >= 'a') && (c <= 'f')) {
    reg |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Serial.print("Input Value to write: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    val[0] = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'f')) {
    val[0] = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    val[0] |= (c - '0');
  } else if ((c >= 'a') && (c <= 'f')) {
    val[0] |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  if(address == 0x10){
    Serial.print("Input High Byte Value to write: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      val[1] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      val[1] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);
  }

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val[0]);
  if(address == 0x10){
    Wire.write(val[1]);
  }
  Wire.endTransmission();

  Serial.print("I2C Write: address: "); Serial.print(address, HEX);
  Serial.print(", register: "); Serial.print(reg, HEX);
  if(address == 0x10){
    Serial.print(", value: "); Serial.print(val[0], HEX); Serial.print(", "); Serial.println(val[1], HEX);
  }
  else {
    Serial.print(", value: "); Serial.println(val[0], HEX);
  }
}

void sub_test_c(void) {

  Serial.println("Sub-test C - ICM42670P");

  char c = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }

    inv_imu_sensor_event_t imu_event;

    // Get last event
    IMU.getDataFromRegisters(&imu_event);

    // Format data for Serial Plotter
    Serial.print(imu_event.accel[0]); // left/right (-2048 ~ 2048)
    Serial.print(",");
    Serial.print(imu_event.accel[1]); // up/down
    Serial.print(",");
    Serial.print(imu_event.accel[2]); // front/back
    Serial.print(",");
    Serial.print(imu_event.gyro[0]);
    Serial.print(",");
    Serial.print(imu_event.gyro[1]);
    Serial.print(",");
    Serial.print(imu_event.gyro[2]);
    Serial.print(",");
    Serial.println(imu_event.temperature);

    // Run @ ODR 100Hz
    delay(1000);
  }
}

void sub_test_d(void) {

  Serial.println("Sub-test D - Button");

  char c = 0;
  uint16_t cnt = 0;
  uint8_t button_status = 0;
  uint16_t bat_status = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    Serial.printf("Input Status (%d) ----------------------\r\n", cnt++);
    button_status = digitalRead(PIN_BOOT);
    Serial.printf("BOOT Button: %d\r\n", button_status);
    button_status = digitalRead(PIN_BUTTON);
    Serial.printf("PUSH Button: %d\r\n", button_status);
    bat_status = analogRead(PIN_BAT);
    Serial.printf("Bat Status: %d (%04x)\r\n", bat_status, bat_status);
    delay(1000);
  }
}

void sub_test_e(void) {

  Serial.println("Sub-test E - VEML3235");

  char c = 0;
  uint8_t data[3] = {0,};
  uint16_t out_als = 0;
  uint16_t out_w = 0;
  int val = 0;

  i2c_write(0x10, 0x00, data, 2);

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    
    // white
    i2c_read_wo(0x10, 0x04, data, 2);
    out_w = 0x100*data[1] + data[0];

    // als 
    i2c_read_wo(0x10, 0x05, data, 2);
    out_als = 0x100*data[1] + data[0];

    val = (int) (((float) out_als)*0.27264);

    Serial.print("ALS: "); Serial.print(out_als); Serial.print(", LUX: "); Serial.print(val);
    Serial.print(", White: "); Serial.println(out_w);

    delay(1000);
  }

}

void sub_test_g(void) {

  Serial.println("Sub-test G - SHT41");

  char c = 0;
  sensors_event_t humidity, temp;
  uint32_t timestamp = millis();

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    
    timestamp = millis();
    sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    timestamp = millis() - timestamp;

    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

    Serial.print("Read duration (ms): ");
    Serial.println(timestamp);

    delay(1000);
  }

}

void sub_test_h(void) {

  Serial.println("Sub-test H - TMF8801");

  char c = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    
    if (tof.isDataReady()) {                    //Is check measuring data vaild, if vaild that print measurement data to USB Serial COM.
      Serial.print("Distance = ");
      Serial.print(tof.getDistance_mm());       //Print measurement data to USB Serial COM, unit mm, in eCOMBINE mode.
      Serial.println(" mm");
    }
  }

}

void sub_test_s(void) {

  char c;
  int r_data = 0;
  String strLog;
  Serial.println("Sub-test S - LiitleFS");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {
    LittleFS.begin();
    LittleFS.format();
  } else if(c == '6') {
    Serial.printf("TotalBytes: %d(0x%x), UsedBytes: %d(0x%x)\r\n", 
      LittleFS.totalBytes(), LittleFS.totalBytes(), LittleFS.usedBytes(), LittleFS.usedBytes());
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_t(void) {

  Serial.println("Sub-test T - L-51POPT1D2");

  char c = 0;
  uint16_t val;
  uint8_t dval;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    //val = analogRead(PIN_PT);
    dval = digitalRead(PIN_PT);
    Serial.print("L-51POPT1D2: "); Serial.println(dval);

    Serial.print("MQ-2: "); Serial.println(analogRead(PIN_MQ2_AO));
    c = 0;
    delay(1000);
  }

}

void sub_test_u(void) {

  Serial.println("Sub-test U - Buzzer");

  char c = 0;
  uint8_t button_status = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println("Quit loop");
      break;
    }
    digitalWrite(PIN_BUZZER, HIGH);
    delay(1000);
  }
}

void sub_test_v(void) {

  Serial.println("Sub-test V - BLE iBeacon");

  char c = 0;
  uint8_t button_status = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    if (deviceConnected) {
      Serial.printf("*** NOTIFY: %d ***\r\n", value);
      pCharacteristic->setValue(&value, 1);
      pCharacteristic->notify();
      value++;
    } else {
      Serial.printf("*** Custom UUID Byte: %02x\r\n", custom_uuid_byte_0);
      update_beacon();
      custom_uuid_byte_0++;
      delay(3000);
    }
    delay(2000);
  }
}

void sub_test_w(void) {

  char c;
  uint8_t id_idx = 0;
  int8_t id_pnt = 0;
  uint8_t id_tmp[5] = {0,};
 
  Serial.println("Sub-test W - Settings");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Clear EEPROM
#if 0
    for(int i = 0; i < EEPROM_SIZE; i++){
      EEPROM.writeByte(i, 0x00);
    }
#else
    memset((void*) &gv_settings, 0x00, sizeof(gv_settings));
    prefs.clear();
#endif
  } else if(c == '1') {  // WLAN settings
    Serial.print("[WLAN] Enter SSID: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      memset(gv_ssid, 0x00, sizeof(gv_ssid));
      strcpy(gv_ssid, cbuf);
      set_eeprom_data_ssid();
      memset(gv_settings.gv_ssid, 0x00, EEPROM_ADDR_SSID_SZ);
      strcpy(gv_settings.gv_ssid, cbuf);
      prefs.putBytes("settings", (void*) &gv_settings, sizeof(gv_settings));
    }

    Serial.print("[WLAN] Enter PASSWD: ");
    memset(cbuf, 0x00, sizeof(cbuf));
    idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      memset(gv_passwd, 0x00, sizeof(gv_passwd));
      strcpy(gv_passwd, cbuf);
      set_eeprom_data_passwd();
      memset(gv_settings.gv_passwd, 0x00, EEPROM_ADDR_PASSWD_SZ);
      strcpy(gv_settings.gv_passwd, cbuf);
      prefs.putBytes("settings", (void*) &gv_settings, sizeof(gv_settings));
    }
  } else if(c == '2') {  // Server settings
    Serial.print("[Server] Enter IP: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      memset(gv_server, 0x00, sizeof(gv_server));
      strcpy(gv_server, cbuf);
      set_eeprom_data_server();
      memset(gv_settings.gv_server, 0x00, EEPROM_ADDR_SERVER_SZ);
      strcpy(gv_settings.gv_server, cbuf);
      prefs.putBytes("settings", (void*) &gv_settings, sizeof(gv_settings));
    }
  } else if(c == '3') {  // ID number settings
    Serial.print("[Device] Enter ID: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(isalnum(cbuf[0])){
      id_idx = 4;
      id_pnt = strlen(cbuf) - 1;
      while(1) {
        if(id_pnt < 1) {
          cbuf[id_pnt] = tolower(cbuf[id_pnt]);
          if((cbuf[id_pnt] >= '0') && (cbuf[id_pnt] <= '9')) {
            id_tmp[id_idx] = (cbuf[id_pnt] - '0');
          } else if ((cbuf[id_pnt] >= 'a') && (cbuf[id_pnt] <= 'f')) {
            id_tmp[id_idx] = (cbuf[id_pnt] - 'a' + 0xa);
          } else {
            Serial.println("Invalid Data String");
            return;
          }
          break;
        } else {
          cbuf[id_pnt-1] = tolower(cbuf[id_pnt-1]);
          if((cbuf[id_pnt-1] >= '0') && (cbuf[id_pnt-1] <= '9')) {
            id_tmp[id_idx] = (cbuf[id_pnt-1] - '0')*0x10;
          } else if ((cbuf[id_pnt-1] >= 'a') && (cbuf[id_pnt-1] <= 'f')) {
            id_tmp[id_idx] = (cbuf[id_pnt-1] - 'a' + 0xa)*0x10;
          } else {
            Serial.println("Invalid Data String");
            return;
          }
          cbuf[id_pnt] = tolower(cbuf[id_pnt]);
          if((cbuf[id_pnt] >= '0') && (cbuf[id_pnt] <= '9')) {
            id_tmp[id_idx] |= (cbuf[id_pnt] - '0');
          } else if ((cbuf[id_pnt] >= 'a') && (cbuf[id_pnt] <= 'f')) {
            id_tmp[id_idx] |= (cbuf[id_pnt] - 'a' + 0xa);
          } else {
            Serial.println("Invalid Data String");
            return;
          }
          id_pnt -= 2;
          id_idx--;
          if(id_pnt < 0) break;
        }
      }

      for(int i = 0; i < 5; i++){
        gv_settings.gv_id[i] = gv_id[i] = id_tmp[i];
      }
      Serial.printf("Input ID: %02x, %02x, %02x, %02x, %02x\r\n", gv_id[0], gv_id[1], gv_id[2], gv_id[3], gv_id[4]);
      set_eeprom_data_sn_num();
      prefs.putBytes("settings", (void*) &gv_settings, sizeof(gv_settings));
    }
  } else if(c == '4') {
#if 0
    uint8_t rdata[32];
    if(isalnum(EEPROM.readByte(EEPROM_ADDR_SSID))){
      memset(rdata, 0x00, sizeof(rdata));
      EEPROM.readBytes(EEPROM_ADDR_SSID, rdata, sizeof(rdata));
      Serial.printf("SSID: %s\r\n", rdata);
    }

    if(isalnum(EEPROM.readByte(EEPROM_ADDR_PASSWD))){
      memset(rdata, 0x00, sizeof(rdata));
      EEPROM.readBytes(EEPROM_ADDR_PASSWD, rdata, sizeof(rdata));
      Serial.printf("PASSWD: %s\r\n", rdata);
    }

    if(isalnum(EEPROM.readByte(EEPROM_ADDR_SERVER))){
      memset(rdata, 0x00, sizeof(rdata));
      EEPROM.readBytes(EEPROM_ADDR_SERVER, rdata, sizeof(rdata));
      Serial.printf("SERVER: %s\r\n", rdata);
    }
#else
    uint8_t rdata[EEPROM_ADDR_SN_NUM_SZ+EEPROM_ADDR_SSID_SZ+EEPROM_ADDR_PASSWD_SZ+EEPROM_ADDR_SERVER_SZ];
    if(prefs.isKey("settings")){
      size_t plen = prefs.getBytesLength("settings");
      Serial.printf("plen: %d\r\n", plen);
      prefs.getBytes("settings", rdata, plen);
      prefs.isKey("settings");
      memcpy((void*)&gv_settings, rdata, plen);
      Serial.printf("ID: %02x, %02x, %02x, %02x, %02x\r\n", gv_settings.gv_id[0], gv_settings.gv_id[1], 
        gv_settings.gv_id[2], gv_settings.gv_id[3], gv_settings.gv_id[4]);
      Serial.printf("SSID: %s\r\n", gv_settings.gv_ssid);
      Serial.printf("PASSWD: %s\r\n", gv_settings.gv_passwd);
      Serial.printf("Server: %s\r\n", gv_settings.gv_server);
    } else {
      Serial.printf("settings is not initialized\r\n");
    }
#endif
  } else {
    Serial.println("Invalid Test Number");
    return;
  }
}
