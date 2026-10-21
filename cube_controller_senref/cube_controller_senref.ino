// Toggle over-the-air (WiFi) programming functionality (0 = disabled, 1 = enabled)
#define OTA 1

// Toggle between use of nonlinear or linear controller (0 = linear, 1 = nonlinear)
#define USE_NONLINEAR_CONTROLLER 1

// --- Ram buffer config(2.5 Minute at 50Hz) ---
#define samples 7000

// TickTwo provides ESP32-compatible timers
#include <TickTwo.h>

// WebSerial and Async Web Server libraries
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#if OTA
  // Libraries related to wireless functionality
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif

// Cube controller library
#include "src/cube-controller.h"


#if OTA
  // WiFi settings
  const char* wifi_ssid = "Livebox7-DF13"; // "LCJ_S22"; "hackerspace_socios"
  const char* wifi_password = "owA5vD2fjXjJ"; //"luiscastillo"; "Siemprelanzacohetes$"
#endif

// Instantiation of objects
Motor motor1(M1_ENABLE, M1_CURRENT), motor2(M2_ENABLE, M2_CURRENT), motor3(M3_ENABLE, M3_CURRENT);
WheelEstimator whe_est_1(M1_SPEED), whe_est_2(M2_SPEED), whe_est_3(M3_SPEED);
AttitudeEstimator att_est(IMU_SDA, IMU_SCL);
AttitudeWheelController cont(USE_NONLINEAR_CONTROLLER);
AttitudeTrajectory att_tra;


// Initialize the web server on port 80
AsyncWebServer server(80);

// Run cube controller at the frequency as specified in parameter file
void controller();
TickTwo timer(controller, dt_us, 0, MICROS_MICROS);

// Status LED control: LED is either solid or blinking at 2.5 Hz
void control_led();
TickTwo timer_led(control_led, 20, 0, MILLIS);

#if OTA
  // Handle OTA flashing requests every 5 seconds
  void handle_ota();
  TickTwo timer_ota(handle_ota, 5000, 0, MILLIS);
#endif

// Quaternion and angle error
float qe0, qe1, qe2, qe3;
float phi;
float phi_lim = phi_min;

// Trajectory initialization flag
bool flag_tra = false;

// LED status when blinking
bool led_status = false;

// Security flags
bool flag_arm = false;
bool flag_terminate = false;

// Torques
float tau_1 = 0, tau_2 = 0, tau_3 = 0;

float start = 0;
float time_m;
float ref;
float omega;
float omega_w;

// Data to save in RAM
struct TelemetryRAM {
  float time_m;
  float ref;
  float phi_rad;
  float omega;
  float omega_w;
  float tau_1;
};

// Buffer instance and flags
TelemetryRAM buffer_ram[samples];
int idx_ram = 0;
bool downloaded = false;
bool armed = false;

// --- Sinusoidal Parameters for the Trajectory ---
const float offset_COM = 0.0 * PI / 180.0;  // COM compensation - Zero gravimetric [rad]
const float amplitud    = 2.0 * PI / 180.0;  // Amplitude (+/- 2 degrees) [rad]
const float frecuencia  = 0.5;               // Frequency [Hz]
volatile float phi_ref;                      // Reference angle [rad]
const float switch_time = 45.0;              // Time at which the sinusoidal trajectory starts [s]  


void setup() {
   
  // Keeping standard Serial initialized just in case, but no longer printing to it
  Serial.begin(921600);

  #if OTA
    // Set up WiFi connection
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);

    unsigned int nAttempts = 0;
    while (WiFi.waitForConnectResult() != WL_CONNECTED && nAttempts < 3) {
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
    } else {
      // WiFi connected successfully! Start the server and WebSerial
      WebSerial.begin(&server);
      server.begin();
      delay(3000);

      // Now that WebSerial is running, we can safely print the boot messages
      WebSerial.println("This is the ESP32 cube controller.");
      WebSerial.print("Connected to WiFi (local IP is ");
      WebSerial.print(WiFi.localIP());
      WebSerial.println(").");
    }

    // Set hostname
    ArduinoOTA.setHostname("Cube ESP32");

    // Set up OTA flashing using ArduinoOTA
    ArduinoOTA.begin();
  #endif

  // Delay for one second to allow Maxon ESCON drivers to fully initialize
  delay(1000);

  // Motor setup (also spins each motor in positive direction very briefly)
  motor1.init();
  motor2.init();
  motor3.init();

  // Wheel estimator initialization (also calibrates the hall sensor for each wheel)
  whe_est_1.init();
  whe_est_2.init();
  whe_est_3.init();
  
  // Attitude estimator initialization (also calibrates the gyroscope)
  att_est.init();

  // Start controller and LED timers
  timer.start();
  timer_led.start();

  #if OTA
    // OTA handler timer
    timer_ota.start();
  #endif

  start = millis();

}

void loop() {

  // Update controller timer
  timer.update();
  timer_led.update();

  #if OTA
    // Handle OTA updates
    timer_ota.update();
  #endif
}

#if OTA
  void handle_ota(){
    ArduinoOTA.handle();
  }
#endif

void controller() {
  // Estimate wheel velocities
  whe_est_1.estimate(tau_1);
  whe_est_2.estimate(tau_2);
  whe_est_3.estimate(tau_3);

  // Estimate cube attitude
  att_est.estimate();

  // =====================================================================================
  // Sinusoidal feedforward trayectory generator
  float t_segundos = (millis() - start) / 1000.0; // current time
  float w = 2.0 * PI * frecuencia;                // angular frequency (omega)
  

  if(t_segundos < switch_time) {
    phi_ref = offset_COM;

    att_tra.qr0 = cos(phi_ref / 2.0 - PI / 8.0);
    att_tra.qr1 = cos(phi_ref / 2.0 + 3.0 * PI / 8.0);
    att_tra.qr2 = 0.0;
    att_tra.qr3 = 0.0;

    // reference velocity     
    att_tra.omega_r_x = 0.0;
    att_tra.omega_r_y = 0.0;
    att_tra.omega_r_z = 0.0;

    // reference acceleration     
    att_tra.alpha_r_x = 0.0;
    att_tra.alpha_r_y = 0.0;
    att_tra.alpha_r_z = 0.0;

  } else {
    phi_ref = offset_COM + amplitud * sin(w * (t_segundos-switch_time));
 
    att_tra.qr0 = cos(phi_ref / 2.0 - PI / 8.0);
    att_tra.qr1 = cos(phi_ref / 2.0 + 3.0 * PI / 8.0);
    att_tra.qr2 = 0.0;
    att_tra.qr3 = 0.0;

    // 2. reference velocity (Derivative of sine = cosine)
    // This tells the controller what speed the cube should be moving at this instant
    att_tra.omega_r_x = 0.0; //amplitud * w * cos(w * (t_segundos-switch_time));
    att_tra.omega_r_y = 0.0;
    att_tra.omega_r_z = 0.0;

    // 3. reference acceleration (Derivative of cosine = -sine)
    // This injects additional torque in advance (Feedforward) to overcome inertia
    att_tra.alpha_r_x = 0.0; //-amplitud * w * w * sin(w * (t_segundos-switch_time));
    att_tra.alpha_r_y = 0.0;
    att_tra.alpha_r_z = 0.0;
  }
    // =====================================================================================



  // Calculate rotation quaternion error
  qe0 = att_est.q0 * att_tra.qr0 + att_est.q1 * att_tra.qr1 + att_est.q2 * att_tra.qr2 + att_est.q3 * att_tra.qr3;
  qe1 = att_est.q0 * att_tra.qr1 - att_est.q1 * att_tra.qr0 - att_est.q2 * att_tra.qr3 + att_est.q3 * att_tra.qr2;
  qe2 = att_est.q0 * att_tra.qr2 - att_est.q2 * att_tra.qr0 + att_est.q1 * att_tra.qr3 - att_est.q3 * att_tra.qr1;
  qe3 = att_est.q0 * att_tra.qr3 - att_est.q1 * att_tra.qr2 + att_est.q2 * att_tra.qr1 - att_est.q3 * att_tra.qr0;

  
  // Normalize rotation quaternion error (real part only since we don't need the rest)
  qe0 /= sqrt(qe0 * qe0 + qe1 * qe1 + qe2 * qe2 + qe3 * qe3);

  // Calculate error angle
  phi = 2.0 * acos(qe0);

  // Controller enable and disable logic. The controller only activates once its orientation is such that the error with
  // respect to the desired orientation is smaller than phi_min. As soon as this is reached, the error tolerance phi_lim
  // is increased to phi_max. If this error is exceeded, the controller is disabled until the chip is reset.
  if(abs(phi) <= phi_lim && !flag_terminate) {
    // Controller is active
    flag_arm = true;

    // Increase error range to the maximum limit
    phi_lim = phi_max;

    /////////////////////////////////////////////////////////////////////////////// IMPORTANT /////////////////////////////////////////////////////////////////////////////

    // Generate trajectory
    // Comment out the generate() method if the cube is balancing on a side instead of a corner
    if(!flag_tra) {
      flag_tra = true;
      att_tra.init();
    }
    // att_tra.generate();
    
    /////////////////////////////////////////////////////////////////////////////// IMPORTANT /////////////////////////////////////////////////////////////////////////////


    // Controller calculates motor torques based on cube and wheel states
    cont.control(att_tra.qr0,
                 att_tra.qr1,
                 att_tra.qr2,
                 att_tra.qr3,

                 att_est.q0,
                 att_est.q1,
                 att_est.q2,
                 att_est.q3,

                 att_tra.omega_r_x,
                 att_tra.omega_r_y,
                 att_tra.omega_r_z,

                 att_est.omega_x,
                 att_est.omega_y,
                 att_est.omega_z,

                 att_tra.alpha_r_x,
                 att_tra.alpha_r_y,
                 att_tra.alpha_r_z,

                 whe_est_1.theta_w,
                 whe_est_2.theta_w,
                 whe_est_3.theta_w,

                 whe_est_1.omega_w,
                 whe_est_2.omega_w,
                 whe_est_3.omega_w
                );

    // Get motor torques from controller
    tau_1 = cont.tau_1;
    tau_2 = cont.tau_2;
    tau_3 = cont.tau_3;
  } else {
    // Outside safe limits: reset motor torques to zero
    tau_1 = 0.0;
    tau_2 = 0.0;
    tau_3 = 0.0;

    // Disarm mechanism
    if(flag_arm) {
      flag_arm = false;
      flag_terminate = true;
    }
  }

  /////////////////////////////////////////////////////////////////////////////// IMPORTANT /////////////////////////////////////////////////////////////////////////////
  // Apply torques to motors
  // When balancing on the x side, comment out motor 2 and 3. When balancing on the y side, comment out 1 and 3.
  motor1.set_torque(tau_1);
  // motor2.set_torque(tau_2);
  // motor3.set_torque(tau_3);
  /////////////////////////////////////////////////////////////////////////////// IMPORTANT /////////////////////////////////////////////////////////////////////////////

}


void control_led() {
  
  time_m = millis() - start;
  omega = att_est.omega_x;
  omega_w = whe_est_1.omega_w;
  ref = phi_ref;

  //  While balancing
  if (flag_arm) {
    armed = true;
    
    if (idx_ram < samples) {
      buffer_ram[idx_ram].time_m = time_m;
      buffer_ram[idx_ram].ref = ref;
      buffer_ram[idx_ram].phi_rad = phi;
      buffer_ram[idx_ram].omega = omega;
      buffer_ram[idx_ram].omega_w = omega_w;
      buffer_ram[idx_ram].tau_1 = tau_1;
      idx_ram++;
    }
  }

  // End of balance
  if (!flag_arm && armed && !downloaded) {
    
    WebSerial.println("--- DOWNLOADING DATA ---");
    WebSerial.println("time,ref,phi,omega_c,omega_w,torque");
    
    for (int i = 0; i < idx_ram; i++) {
      char buffer_out[128];
      sprintf(buffer_out, "%.0f, %.4f, %.4f, %.4f, %.4f, %.4f\r\n", 
              buffer_ram[i].time_m, 
              buffer_ram[i].ref, 
              buffer_ram[i].phi_rad, 
              buffer_ram[i].omega, 
              buffer_ram[i].omega_w, 
              buffer_ram[i].tau_1);
              
      WebSerial.print(buffer_out);
      
      // delay for buffer to be downloaded to WebSerial
      delay(100); 
    }
    
    WebSerial.println("--- DOWNLOAD COMPLETED ---");
    downloaded = true;
  // =====================================================================================
  }

  
  if(flag_arm) {
    if(abs(phi) <= phi_lim - 10 * pi / 180) {
      // Cube is not close to error limit: solid green LED
      // WebSerial.println("Cube is not close to error limit. Phi: " + String(phi));

      neopixelWrite(RGB_BUILTIN, 0, 255, 0);
    } else {
      // Cube is close to error limit: alternate between red and green. Cube should be manually put to rest.
      // WebSerial.println("Error limit approached. Phi: " + String(phi));

      if(led_status) {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
      } else {
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
      }
      led_status = !led_status;
    }
  } else {
    if(flag_tra) {
      // A trajectory was already generated so the cube must now be halted due to a limit error
      // WebSerial.println("Cube is halted due to error limit breach. Phi: " + String(phi));

      if(led_status) {
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
      } else {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
      }
      led_status = !led_status;
    } else {
      // Show a solid red LED to indicate readiness to start
      // WebSerial.println("Cube is ready to start. Phi: " + String(phi));
        
      neopixelWrite(RGB_BUILTIN, 255, 0, 0);
    }
  }
}