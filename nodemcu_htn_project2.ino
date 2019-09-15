#include <Arduino.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <Wire.h>

// // TRIPLE AXIS ACCELEROMETER SETUP
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication
const uint8_t scl = D6;
const uint8_t sda = D7;

// sensitivity scale factor respective to full scale setting provided in
// datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY,
    GyroZ;  // accelerometer values
double pitch, roll;
//
// // VAR SETUP END

// // Touch Sensor Setup
//
const int sensorPin = A0;  // the input pin for the touch sensor
int sensorValue_top =
    0;                  // variable to store the value coming from touch sensor
const float VCC = 3.3;  //  voltage of ESP8266 3.3V pinout
const float total_resistance =
    3000.0;  //  resistance of three 1k resistors in series
//
// // Var setup END

// // GET Request Setup
//
ESP8266WiFiMulti WiFiMulti;
double duration = 0;  // our time that is sent in GET request
bool timer =
    false;  // simple boolean to keep track of whether ticker is rolling
unsigned long ticker = 0;  // time in milliseconds
// below WIFI information is hard-coded in
const char* ssid = "Hack the North";  // The SSID (name) of the Wi-Fi network
                                      // you want to connect to
const char* password = "uwaterloo";   // The password of the Wi-Fi network
//
// // Var setup END

// // Side Function Definitions
//
void getAngle(double Vx, double Vy, double Vz) {
  // convert the accel data to pitch/roll
  double x = Vx;
  double y = Vy;
  double z = Vz;

  pitch = atan(x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  // convert radians into degrees
  pitch = pitch * (180.0 / 3.14);
  roll = roll * (180.0 / 3.14);
  // uses pitch to determine juul/vape orientation
}

float getForce(int analogInput) {
  // If the force sensitive resistor has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.

  // Use analogInput to calculate voltage:
  float voltage = analogInput * VCC / 1023.0;
  // Use voltage and static resistor value to
  // calculate FSR resistance:
  float resistance = total_resistance * (VCC / voltage - 1.0);
  //    Serial.println("Resistance: " + String(fsrR) + " ohms");
  // Guesstimate force based on slopes in figure 3 of
  // FSR datasheet:
  float force;
  float conductance = 1.0 / resistance;  // Calculate conductance
  // Break parabolic curve down into two linear slopes:
  if (conductance <= 600.0) {
    force = (conductance - 0.00075) / 0.00000032639;
  } else {
    force = conductance / 0.000000642857;
  }

  return force;  // force above threshold registers as 'button press'
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  // read all 14 register
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

void MPU6050_Init() {
  // configure MPU6050
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG,
            0x00);  // set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG,
            0x00);  // set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

void sendDuration() {
  // wait for WiFi connection
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    HTTPClient http;
    Serial.print("[HTTP] begin...\n");
    // configure traged server and url
    http.begin("http://vape-publisher.herokuapp.com/publish?duration=" +
               String(duration) + ":00");  // sends ticker data in GET request
    Serial.print("[HTTP] GET...\n");
    // start connection and send HTTP GET
    int httpCode = http.GET();
    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP has been sent in proper manner, server responds favorably
      Serial.printf("[HTTP] GET... code: %d\n", httpCode);
      Serial.println("Duration: " +
                     String(duration));  // for confirmation purposes
      // file found at server
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println(payload);
      }
    } else {  // failed requests
      Serial.printf("[HTTP] GET... failed, error: %s\n",
                    http.errorToString(httpCode).c_str());
    }
    http.end();
  }
}
//
// // Non-main functions END

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);  // analog pin open to data
  Wire.begin(sda, scl);       // reads accelerometer data pins
  MPU6050_Init();             // configures accel

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();  // general precaution, not obligatory
    delay(1000);
  }

  WiFiMulti.addAP(ssid, password);  // logs into WIFI as client
}

void loop() {  // main function

  // below section reads accel values
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  // calculate accel variables and displays them
  Ax = (double)AccelX / AccelScaleFactor;
  Ay = (double)AccelY / AccelScaleFactor;
  Az = (double)AccelZ / AccelScaleFactor;
  T = (double)Temperature / 340 + 36.53;  // temperature formula
  Gx = (double)GyroX / GyroScaleFactor;
  Gy = (double)GyroY / GyroScaleFactor;
  Gz = (double)GyroZ / GyroScaleFactor;
  getAngle(Ax, Ay, Az);
  Serial.print("Angle: ");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print(" | Roll = ");
  Serial.println(roll);

  // below reads analog values
  sensorValue_top = analogRead(sensorPin);
  float top_force = getForce(sensorValue_top);           // calculate forces
  Serial.println("Force: " + String(top_force) + " g");  // approximate grams

  // below is the procedure for when sensors indicate vaping is occuring
  if (abs(top_force) > 100000 && timer == false &&
      (pitch < -10 &&
       pitch > -90))  // first sign of vaping includes a certain angle of hold,
                      // and fingers on the manual button
  {
    timer = true;       // indicates ticker now started
    ticker = millis();  // millis at that time
    delay(50);          // decrease checking delay
  } else if (abs(top_force) > 100000 && timer == true &&
             (pitch < -10 && pitch > -90)) {  // consequential signs of vaping
    duration = millis() - ticker;  // current minus past = time elapsed
    Serial.println("Button pressed duration: " + String(duration));
    delay(50);
  } else {       // no more signs- vaping over
    ticker = 0;  // reset variables
    timer = false;
    if (duration != 0) {
      // send duration
      sendDuration();
      duration = 0;
    }
    delay(400);  // standard cycle delay
  }
}
