// This program can be used to get the offsets and scaling factors for the gyro and accelerometer. This cannot be done
// automatically. The cube must be placed in six different orientations, corresponding to each axis facing up and down.
// Calibration offsets should initially be set to zero. Use the methodology as described in Analog Devices AN-1057,
// which may be found here: https://www.analog.com/en/app-notes/an-1057.html. In particular, equations 17 and 18.
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
/////////////////////////////////////////



#include <Wire.h>
#include <ICM20948_WE.h>

#include <WiFi.h>
#include <ArduinoOTA.h>

// TickTwo provides ESP32-compatible timers
#include <TickTwo.h>

// Toggle over-the-air (WiFi) programming functionality (0 = disabled, 1 = enabled)
#define OTA 1

#define ICM20948_ADDR 0x69

#if OTA
  // Libraries related to wireless functionality
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif

#if OTA
  // WiFi settings
  const char* wifi_ssid = "Livebox7-DF13"; // "LCJ_S22"; 
  const char* wifi_password = "owA5vD2fjXjJ"; //"luiscastillo";
#endif

#if OTA
  // Handle OTA flashing requests every 5 seconds
  void handle_ota();
  TickTwo timer_ota(handle_ota, 5000, 0, MILLIS);
#endif

TwoWire wirePort = TwoWire(0);
ICM20948_WE myIMU = ICM20948_WE(&wirePort, ICM20948_ADDR);


unsigned long lastLogTime = 0;



//////////////////////////////////////////////////
// Initialize the web server on port 80
AsyncWebServer server(80);
////////////////////////////////////////////



void setup() {
  // Set up serial
  Serial.begin(921600);

  #if OTA
    // Set up WiFi connection
    WiFi.mode( WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);

    unsigned int nAttempts = 0;
    while (WiFi.waitForConnectResult() != WL_CONNECTED && nAttempts < 10) {
      delay(1000);
      nAttempts++;
    }

        // If WiFi connection failed, indicate this using six yellow flashes
    if(WiFi.waitForConnectResult() != WL_CONNECTED) {
      for(int i = 0; i < 5; i++) {
        neopixelWrite(RGB_BUILTIN, 255, 255, 0);
        delay(250);
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
        delay(250);
      }
    }

        // Set hostname
    ArduinoOTA.setHostname("Cube ESP32");

    // Print WiFi status
    Serial.print("Connected to WiFi (local IP is ");
    Serial.print(WiFi.localIP());
    Serial.println(").");

    // Set up OTA flashing using ArduinoOTA
    ArduinoOTA.begin();
  #endif


  // Set up I2C on pins 47 (SDA) and 21 (SCL) at 400 kHz
  wirePort.begin(47, 21, 400000);

  // Initialize IMU
  if (!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  } else {
    Serial.println("ICM20948 is connected");
  }

  // Set accelerometer and gyroscope ranges
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_2000);

  // Set digital low pass filter setting such that noise is minimized (lowest cut-off frequency)
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  // Disable sample rate divider
  myIMU.setAccSampleRateDivider(0);
  myIMU.setGyrSampleRateDivider(0);

    #if OTA
    // OTA handler timer
    timer_ota.start();
  #endif

  //////////////////////////////////////////
  // Initialize WebSerial
  WebSerial.begin(&server);
 
  // Start the web server
  server.begin();




}

unsigned long timer;
xyzFloat acc, gyr;
float ax, ay, az, gx, gy, gz;

// IMU gyroscope offsets
const float b_gyr_x = 0.670; // *
const float b_gyr_y = -0.0647; // *
const float b_gyr_z = 0.361; // *

// IMU accelerator offsets and gains
const float b_acc_x = -127.65; // *
const float b_acc_y = -425.66; // *
const float b_acc_z = -382.27;  // *

const float f_acc_x = 1671.81; // *
const float f_acc_y = 1672.20; // *
const float f_acc_z = 1706.44; // *



void loop() {

  #if OTA
    // Handle OTA updates
    timer_ota.update();
  #endif

  


  // Start a timer
  timer = millis();

  // Clear accumulation registers before starting the loop
  ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;

  // Measure 1000 samples and average them
  for (int i = 0; i < 1000; i++) {
    myIMU.readSensor();
    acc = myIMU.getAccRawValues();
    gyr = myIMU.getGyrValues();

    ax += acc.x / 1000;
    ay += acc.y / 1000;
    az += acc.z / 1000;
    gx += gyr.x / 1000;
    gy += gyr.y / 1000;
    gz += gyr.z / 1000;



    // Print raw and calibrated values in last cycle
    if (i == 999) {
      
      // 1. Build the Raw Data String
      String rawMsg = "Raw: Acc: " + String(ax, 4) + " _ " + String(ay, 4) + " _ " + String(az, 4) + 
                      " | gyr: " + String(gx, 4) + " _ " + String(gy, 4) + " _ " + String(gz, 4);
      
      // Send it in ONE packet
      WebSerial.println(rawMsg);

      // 2. Build the Calibrated Data String
      String calMsg = "Cal: Acc: " + String((ax - b_acc_x) / f_acc_x, 4) + " _ " + 
                      String((ay - b_acc_y) / f_acc_y, 4) + " _ " + String((az - b_acc_z) / f_acc_z, 4) + 
                      " | gyr: " + String(gx - b_gyr_x, 4) + " _ " + String(gy - b_gyr_y, 4) + " _ " + String(gz - b_gyr_z, 4);
      
      // Send it in ONE packet
      WebSerial.println(calMsg);
    }
  }

  // 3. Build and send the timing string in ONE packet
  String timeMsg = "Took " + String(millis() - timer) + " ms.";
  // WebSerial.println(timeMsg);

  // Reset averages
  ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;

  delay(500);
}

#if OTA
  void handle_ota(){
    ArduinoOTA.handle();
  }
#endif
