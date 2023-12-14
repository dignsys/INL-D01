# INL-D01

## Install Arduino IDE
- Download Arduino IDE

  참조: <https://www.arduino.cc/en/software>
- Install Additional Board Manager URLs for ESP32

  Arduino IDE : File --> Preferences --> Addtional board manager URLs에 하기의 json 파일 경로를 추가

  - Stable release link
  ```
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
  ```

  참조: <https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html>

## Setup Board and Libraries
- Setup Board Type

  Arduino IDE : Tools --> Board --> esp32 --> ESP32S3 Dev Module 선택

- Standard Arduino Library 추가

  Arduino IDE : Tools --> Manage Libraries... --> Library Manager

  Install이 필요한 Library : Ethernet_Generic, Adafruit_SHT4x, ICM42670P, DFRobot_TMF8x01

- User Arduino Library (.ZIP) 추가

  Arduino IDE : Tools --> Sketch --> Include Library --> Add .ZIP Libraries...

- Partition Scheme 선택

  Flash Memory의 Application 영역을 충분히 크게 확보하기 위해서, 하기와 같이 Partition Scheme을 선택한다. Default Partition 선택시, Application 영역이 부족하여 빌드 오류가 발생할 수 있다.
  
  Arduino IDE : Tools --> Partition Scheme --> Minimal SPIFF (1.9MB APP with OTA/190KB SPIFFS)
## Sample Source Code for Hardware Basic Operation
- 실행 방법
  시스템을 부팅하면, 각 센서의 값을 측정하여 주기적으로 BLE Advertising를 송출하는 기능이 Main Loop로 실행됩니다. 이후, '#'키를 입력하여, Sub Test 함수들을 시험할 수 있는 메뉴로 진입합니다.

  각 시험 항목들은 sub_test_*() 함수로 구분되어 작성되어 있고, Serial Monitor의 Message 창에 해당하는 함수의 명령어를 입력하여 실행할 수 있습니다. 예를 들어, sub_test_a( )는 'a'를 입력하고 'Enter'키를 입력함으로써, 실행됩니다.

  Sub Test 메뉴에서 다시 Main Loop로 돌아가고자 한다면, 'x'키를 눌러 Main Loop를 실행할 수 있습니다.

- Sub Test Function List
  - sub_test_a( ) : Read I2C - I2C channel에 연결된 디바이스를 읽기 위한 함수
  - sub_test_b( ) : Write I2C - I2C channel에 연결된 디바이스를 쓰기 위한 함수
  - sub_test_c( ) : ICM42670P - 가속도 및 자이로 측정을 주기적으로 확인하기 위한 함수
  - sub_test_d( ) : Button - BOOT 버튼 및 긴급 버튼의 동작을 확인하기 위한 함수
  - sub_test_e( ) : VEML3235 - 조도 측정을 주기적으로 확인하기 위한 함수
  - sub_test_g( ) : SHT41 - 온/습도 측정을 주기적으로 확인하기 위한 함수
  - sub_test_h( ) : TMF8801 - TOF를 통한 거리 측정을 주기적으로 확인하기 위한 함수
  - sub_test_t( ) : L-51POPT1D2 - 불꽃 감지 및 가스 측정을 주기적으로 확인하기 위한 함수
  - sub_test_u( ) : Buzzer - Buzzer 동작을 확인하기 위한 함수
  - sub_test_v( ) : BLE iBeacon - BLE iBeacon의 Connection과 Advertising을 확인하기 위한 함수

