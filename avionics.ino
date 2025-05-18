#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>
#include <ESP32Servo.h> // Include the ESP32Servo library

// Define constants
#define SDA_PIN 21
#define SCL_PIN 22
#define CS_PIN 5
#define MOS_1 13
#define MOS_2 13                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

// Create TwoWire instance for the I2C 
TwoWire I2C1 = TwoWire(0); // First I2C bus

Adafruit_BMP085 bmp; // Create BMP180 object
File dataFile;
char fileName[] = "/datalog.txt";

// Hardware serial definitions
const int XBEE_RX = 1;
const int XBEE_TX = 3;

HardwareSerial XBee(0); // For XBee communication
HardwareSerial GNSS(2); // For GNSS communication

Servo myServo; // Create a servo object

// Variables for sensor readings and recovery logic
float referenceHeight = 0.0;
float maxHeight = 0.0;
bool mos1Activated = false;
unsigned long mos1ActivatedTime = 0;
String timestr, latitudeStr, longitudeStr;

void setup() {
  // Configure pins
  pinMode(MOS_1, OUTPUT);
  pinMode(MOS_2, OUTPUT);
 // digitalWrite(MOS_1, LOW);
  //digitalWrite(MOS_2, LOW);

  // Initialize serial communication
  XBee.begin(9600, SERIAL_8N1, XBEE_RX, XBEE_TX);
  GNSS.begin(9600, SERIAL_8N1, 16, 17);
  Serial.begin(9600);
  Serial.println("Transmitter ready");

  // Initialize I2C bus
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  Serial.println("I2C initialized successfully");

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 sensor, check wiring!");
    while (1);
  }

  // Initialize SD card
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD Card initialization failed!!!");
  }

  // Attach the servo to a pin (e.g., pin 23 for ESP32)
  myServo.attach(13);
  // Initialize the servo position
  myServo.write(0); // Start at 0 degrees

  // Calculate the reference height
  unsigned long sumHeight = 0;
  int count = 0;

  while (count < 100) {
    sumHeight += bmp.readAltitude();
    count++;
  }
  referenceHeight = sumHeight / count;

  // Open or create the data file on the SD card
  dataFile = SD.open(fileName, FILE_APPEND);
  if (dataFile) {
    dataFile.println("Start");
    dataFile.close();
    Serial.println("Writing to SD card");
  } else {
    Serial.println("Error opening data file.");
  }

  Serial.println("BMP180 initialized successfully");
}

void loop() {
  // Read sensor data
  float rawHeight = bmp.readAltitude();
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float height = rawHeight - referenceHeight;

  // GPS data parsing
  String timestr, latitudeStr, longitudeStr;
  while (GNSS.available()) {
    String gpsData = GNSS.readStringUntil('\n');
    if (gpsData.startsWith("$GNGGA")) {
      int comma1 = gpsData.indexOf(',');
      int comma2 = gpsData.indexOf(',', comma1 + 1);
      int comma3 = gpsData.indexOf(',', comma2 + 1);
      int comma4 = gpsData.indexOf(',', comma3 + 1);
      int comma5 = gpsData.indexOf(',', comma4 + 1);

      timestr = gpsData.substring(comma1 + 1, comma2);
      timestr = timestr.substring(0, 2) + ":" + timestr.substring(2, 4) + ":" + timestr.substring(4, 6);

      latitudeStr = gpsData.substring(comma2 + 1, comma3);
      longitudeStr = gpsData.substring(comma4 + 1, comma5);
    }
  }

  // Recovery system logic
  if (height - maxHeight < -3) {
    digitalWrite(MOS_1, HIGH);
    Serial.println("Recovery Logic Activated");
    if (!mos1Activated) {
      mos1ActivatedTime = millis();
      mos1Activated = true;

      // Rotate the servo 90 degrees
      myServo.write(94);
      Serial.println("Servo rotated 90 degrees");
      delay (10000);
    }
  } else {
    maxHeight = height;
  }
  if (mos1Activated && millis() - mos1ActivatedTime >= 3000) {
    digitalWrite(MOS_2, HIGH);
    Serial.println("Payload Logic Activated");
  }

  // Prepare data packet
  String datapkt = timestr + "," + latitudeStr + "," + longitudeStr + "," + String(height) + "," + String(temp) + "," + String(pressure);
  Serial.println(datapkt);

  // Write to SD card
  dataFile = SD.open(fileName, FILE_APPEND);
  if (dataFile) {
    dataFile.println(datapkt);
    dataFile.close();
  }

  //delay(100); // Small delay to prevent rapid execution
}