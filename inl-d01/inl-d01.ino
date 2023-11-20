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

#define DISABLE_BLE_OP    0
#define BLE_FORMAT_TEST   0

#define DEVICE_NAME            "ESP32"
#define SERVICE_UUID           "7A0247E7-8E88-409B-A959-AB5092DDB03E"
#define BEACON_UUID            "2D7A9F0C-E0E8-4CC9-A71B-A21DB2D034A1"
//#define BEACON_UUID_REV        "88888888-1DA2-1BA7-C94C-E8E00C9F7A2D"
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
uint16_t gv_gas = 0;
int8_t gv_temperature = 0;
uint8_t gv_humidity = 0;
uint16_t gv_illuminance = 0;
uint16_t gv_tof = 0;

void init_service(void);
void init_beacon(void);
void update_beacon(void);
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

#define MQ2_WARM_UP_TIME    300000  //1000*60*5  // 5 minutes

int mq2_warm_up_end = 0;
unsigned long sys_start_millis = 0;

enum {
  STATE_FLAME_NONE,
  STATE_FLAME_DETECTED,
};

// put function declarations here:
void sub_test_a(void);
void sub_test_b(void);
void sub_test_c(void);
void sub_test_d(void);
void sub_test_e(void);
void sub_test_g(void);
void sub_test_h(void);
void sub_test_s(void);
void sub_test_t(void);
void sub_test_u(void);
void sub_test_v(void);
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

void isr_button(void);
void set_buzzer(int state);

uint8_t get_flame(void);
int16_t get_axis_x(void);
int16_t get_axis_y(void);
int16_t get_axis_z(void);
uint16_t get_gas(void);
int8_t get_temperature(void);
uint8_t get_humidity(void);
uint16_t get_illuminance(void);
uint16_t get_tof(void);

void setup() {

  int ret;
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);  // 1MHz, 400kHz, 100kHz

  // Arduino USART setup.
  Serial.begin(115200);

  pinMode(PIN_PT, INPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

#if 1
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
#if (DISABLE_BLE_OP != 1) // Disable BLE init
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

}

void loop() {

  Serial.println();
  Serial.println("INL-D01 Main Loop");
  Serial.println("(C) 2023 Dignsys");
  Serial.println();

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
#if (DISABLE_BLE_OP != 1) // Disable BLE
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
        update_beacon();
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
      if((cur_millis - prev_millis) < 500){
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
  Serial.println();

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

void sub_task_flame(void) {

  gv_flame = digitalRead(PIN_PT);
  Serial.printf("Flame Sensor: %d\r\n", gv_flame);

}

void sub_task_three_axis(void) {

#if 1
  inv_imu_sensor_event_t imu_event;

  IMU.getDataFromRegisters(&imu_event);

  gv_axis_x = imu_event.accel[0];
  gv_axis_y = imu_event.accel[1];
  gv_axis_z = imu_event.accel[2];
  Serial.printf("3 Axis Sensor: %d, %d, %d\r\n", gv_axis_x, gv_axis_y, gv_axis_z);
#else
  gv_axis_x = 120;
  gv_axis_y = 10;
  gv_axis_z = -10;
#endif
}

void sub_task_gas(void) {

#if 1
  gv_gas = analogRead(PIN_MQ2_AO);

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
#else
  gv_gas = 320;
#endif
}

void sub_task_temperature(void) {

  sensors_event_t humidity, temp;

  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  gv_temperature = (int8_t) temp.temperature;
  gv_humidity = (uint8_t) humidity.relative_humidity;

  Serial.printf("Temperature: %d, Humidity: %d\r\n", gv_temperature, gv_humidity);

}

void sub_task_illuminance(void) {

  uint8_t data[3] = {0,};
  uint16_t out_als = 0;
  uint16_t out_w = 0;
  int val = 0;
  static uint8_t mes_start = 1;

  if(mes_start){
    mes_start = 0;
    i2c_write(0x10, 0x00, data, 2);
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

  if (tof.isDataReady()) {
    gv_tof = tof.getDistance_mm();
  }
  Serial.printf("TOF Sensor: %d\r\n", gv_tof);
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

uint16_t get_gas(void) {

#if (BLE_FORMAT_TEST == 1)
  return 320;
#else
  return gv_gas;
#endif
}

int8_t get_temperature(void) {

#if (BLE_FORMAT_TEST == 1)
  return 27;
#else
  return gv_temperature;
#endif
}

uint8_t get_humidity(void) {

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

  uint8_t data;
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
