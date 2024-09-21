#include <Wire.h>               // Include the Wire library for I2C communication
#include <LiquidCrystal_I2C.h>  // Include the LiquidCrystal_I2C library for LCD display
#include <DHT.h>                // Include the DHT library for DHT22 sensor
#include <LTR390.h>             // Include the LTR390 library for the light and UV sensor
#include <Adafruit_BMP280.h>    // Include the Adafruit BMP280 library for pressure and altitude sensor

#define DHTPIN 2           // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22      // DHT sensor type
DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor

const int MQ9_PIN = A0;           // Analog pin connected to the MQ-9 sensor
const int RAIN_PIN = A1;          // Analog pin connected to the Rain Drop sensor
const int RAIN_THRESHOLD = 700;   // Define the rain threshold value
const int RAIN_MAX_VALUE = 4095;  // Maximum value of the rain sensor

#define I2C_LCD_ADDRESS 0x27     // I2C address for the LCD
#define I2C_LTR390_ADDRESS 0x53  // I2C address for the LTR390 sensor
#define TCAADDR 0x70             // TCA9548A I2C address

LiquidCrystal_I2C lcd(I2C_LCD_ADDRESS, 16, 2);  // Set the LCD address, columns, and rows
LTR390 ltr390(I2C_LTR390_ADDRESS);              // Initialize the LTR390 sensor
Adafruit_BMP280 bmp280;                         // Initialize the BMP280 sensor

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

float calculateDewPoint(float temperature, float humidity) {
  // Constants for the Magnus formula
  const float a = 17.27;
  const float b = 237.7;

  // Magnus formula
  float alpha = ((a * temperature) / (b + temperature)) + log(humidity / 100.0);
  float dewPoint = (b * alpha) / (a - alpha);
  return dewPoint;
}

void setup() {
  Serial.begin(9600);  // Start serial communication
  Wire.begin();        // Initialize I2C communication

  // Start sensor initializations
  dht.begin();  // Start DHT sensor

  // Select the channel for LCD
  tcaSelect(1);     // Assume LCD is on channel 1
  lcd.init();       // Initialize LCD
  lcd.backlight();  // Turn on backlight

  // Select the channel for LTR390
  tcaSelect(0);  // Assume LTR390 is on channel 0
  if (!ltr390.init()) {
    Serial.println("LTR390 not connected!");
  }
  ltr390.setMode(LTR390_MODE_ALS);
  ltr390.setGain(LTR390_GAIN_3);
  ltr390.setResolution(LTR390_RESOLUTION_18BIT);

  // Select the channel for BMP280
  tcaSelect(2);  // Assume BMP280 is on channel 2
  if (!bmp280.begin(0x76)) {
    Serial.println("BMP280 not connected!");
  }
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // End sensor initializations
}

void loop() {
  delay(1000);  // Delay between readings

  // Read data from analog and digital pins
  float temperature = dht.readTemperature();  // Read temperature
  int humidity = dht.readHumidity();          // Read humidity

  float mq9Value = analogRead(MQ9_PIN);         // Read value from MQ-9 sensor
  float voltage = mq9Value * (3.3 / 4095.0);    // Convert analog value to voltage
  float mq9_ppm = (voltage - 0.1) * 100 / 0.8;  // Convert voltage to ppm (Parts Per Million)

  int rainValue = analogRead(RAIN_PIN);                            // Read value from Rain Drop sensor
  int rainPercentage = map(rainValue, 0, RAIN_MAX_VALUE, 100, 0);  // Map the rain value to a percentage

  if (isnan(temperature) || isnan(humidity)) {  // Check if any DHT reading failed
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  float dewPoint = calculateDewPoint(temperature, humidity);

  // Read data from LTR390 using TCA9548A
  tcaSelect(0);  // Select channel 0 for LTR390
  float lux = 0;
  float uvi = 0;
  if (ltr390.newDataAvailable()) {
    if (ltr390.getMode() == LTR390_MODE_ALS) {
      lux = ltr390.getLux();
      ltr390.setGain(LTR390_GAIN_18);                 // Recommended for UVI - x18
      ltr390.setResolution(LTR390_RESOLUTION_20BIT);  // Recommended for UVI - 20-bit
      ltr390.setMode(LTR390_MODE_UVS);
      delay(100);  // Small delay to allow mode switch
      uvi = ltr390.getUVI();
      ltr390.setGain(LTR390_GAIN_3);                  // Recommended for Lux - x3
      ltr390.setResolution(LTR390_RESOLUTION_18BIT);  // Recommended for Lux - 18-bit
      ltr390.setMode(LTR390_MODE_ALS);
    }
  }

  // Read data from BMP280 using TCA9548A
  tcaSelect(2);  // Select channel 2 for BMP280
  float pressure = bmp280.readPressure() / 1000;  // Convert to hPa
  float altitude = bmp280.readAltitude(1013.25);    // Calculate altitude with a baseline pressure

  // Print data to Serial
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C\t");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("CO PPM: ");
  Serial.print(mq9_ppm);
  Serial.println(" ppm");

  Serial.print("Rain Drop Value: ");
  Serial.println(rainValue);
  Serial.println(" %");

  Serial.print("Ambient Light: ");
  Serial.print(lux);
  Serial.println(" Lux");

  Serial.print("UV Index: ");
  Serial.println(uvi);

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" kPa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  
  Serial.print("Dew: ");
  Serial.print(dewPoint);
  Serial.println(" °C");
  
  // Select the channel for LCD
  tcaSelect(1);  // Select channel 1 for LCD

  // Display data on LCD
  lcd.clear();  // Clear the LCD display

  lcd.setCursor(0, 0);  // Set cursor to the first column and first row
  lcd.print("Temp:");
  lcd.print(temperature);
  lcd.print("C");

  lcd.setCursor(0, 1);  // Set cursor to the first column and second row
  lcd.print("Humi:");
  lcd.print(humidity);
  lcd.print("%");

  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);  // Set cursor to the first column and first row
  lcd.print("CO:");
  lcd.print(mq9_ppm);
  lcd.print("ppm");

  lcd.setCursor(0, 1);  // Set cursor to the first column and second row
  lcd.print("Rain:");
  lcd.print(rainPercentage);
  lcd.print("%");

  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);  // Set cursor to the first column and first row
  lcd.print("Lux:");
  lcd.print(lux);
  lcd.print("lux");

  lcd.setCursor(0, 1);  // Set cursor to the first column and second row
  lcd.print("UV Index:");
  lcd.print(uvi);

  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);  // Set cursor to the first column and first row
  lcd.print("Press:");
  lcd.print(pressure);
  lcd.print("kPa");

  lcd.setCursor(0, 1);  // Set cursor to the first column and second row
  lcd.print("Alt:");
  lcd.print(altitude);
  lcd.print("m");

  delay(2000);
  
  lcd.clear();
  lcd.setCursor(0, 0);  // Set cursor to the first column and first row
  lcd.print("Dew:");
  lcd.print(dewPoint);
  lcd.print("C");

  delay(2000);
}
