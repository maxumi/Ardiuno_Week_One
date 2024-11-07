/**
 * @file Run_File.ino
 * @brief This is my ardiuno file for running the script
 */
#include <DHT11.h>
#include <Bounce2.h>
#include <EEPROM.h>

/// @name Pin Definitions
/// Pins for sensor, button, potentiometer, and LEDs
/// @{
#define DHT_PIN 2           ///< pin for the DHT11 temperature sensor
#define BUTTON_PIN 4        ///< Pin for button to toggle between pause state
#define POT_PIN A0          ///< An Analog pin for potentiometer to change target
#define INTERVAL_IN_MS 5    ///< Debounce interval in milliseconds for pressing the button
/// @}

/// @name LED Lights
/// LEDs of differing states
/// @{
#define COLD_LED 10         ///< Blue LED for below target temperature
#define OK_LED 9            ///< White LED for target or close to it.
#define WARM_LED 8          ///< Red LED for above target temperature
#define PAUSE_LED 7         ///< Yellow LED for showing paused state if required
/// @}

/// Instances
DHT11 dht11(DHT_PIN);       ///< DHT11 sensor instance so it can read temperature and humidity
Bounce2::Button button;     ///< Button instance for debounce.

/// Variables
bool isPaused = false;      ///< if DHT11 is paused
unsigned long lastReadingTime = 0; ///< Last timestamp when temperature was read
const unsigned long readingInterval = 1000; ///< Interval for temperature readings (ms)
int targetTemperature = 22; ///< Default target temperature before being adjusted by potentiometer

/// Temperature limits
const int minTemp = 15;     ///< Minimum temperature
const int maxTemp = 50;     ///< Maximum temperature

/**
 * @brief Sets up the thermostat system.
 * 
 * Initializes the pin modes for LEDs, button, starts serial communication for printing,
 * Attaches and configures the button with debounce settings, and loads 
 * Reads target temperature and paused state from EEPROM (Memory) to retain settings after a restart.
 */
void setup() {
  pinMode(COLD_LED, OUTPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(WARM_LED, OUTPUT);
  pinMode(PAUSE_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  // Attach button with debounce settings
  button.attach(BUTTON_PIN, INPUT_PULLUP);
  button.interval(INTERVAL_IN_MS);
  button.setPressedState(LOW);

  // Load target temperature from EEPROM
  EEPROM.get(0, targetTemperature);
  targetTemperature = constrain(targetTemperature, minTemp, maxTemp); // Ensure valid range

  // Load paused state from EEPROM and update the pause LED
  EEPROM.get(sizeof(targetTemperature), isPaused);
  digitalWrite(PAUSE_LED, isPaused ? HIGH : LOW);
}

/**
 * @brief Main loop controlling the thermostat functionality.
 */
void loop() {
  button.update();

  // Handle button press to toggle pause
  if (button.pressed()) {
    isPaused = !isPaused;
    digitalWrite(PAUSE_LED, isPaused ? HIGH : LOW);  // Indicate paused state with yellow LED

    // Store the paused state in EEPROM when changed
    EEPROM.put(sizeof(targetTemperature), isPaused);

    // Turn off all temperature LEDs if the system is paused
    if (isPaused) {
      digitalWrite(COLD_LED, LOW);
      digitalWrite(OK_LED, LOW);
      digitalWrite(WARM_LED, LOW);
    }
  }

  // Only adjust target temperature and take readings when not paused
  if (!isPaused) {
    // Read and map potentiometer value to set target temperature within a range (Cant go above or below)
    int potValue = analogRead(POT_PIN);
    targetTemperature = map(potValue, 0, 1023, minTemp, maxTemp);

    // Store the target temperature in EEPROM only if it has changed significantly
    int previousTargetTemperature;
    EEPROM.get(0, previousTargetTemperature);
    if (abs(previousTargetTemperature - targetTemperature) >= 1) { 
      EEPROM.put(0, targetTemperature);
    }

    unsigned long currentTime = millis();
    // Read temperature and humidity at the specified interval (Don't want to print too much)
    if (currentTime - lastReadingTime >= readingInterval) {
      lastReadingTime = currentTime;

      int temperature, humidity;
      int result = dht11.readTemperatureHumidity(temperature, humidity);

      // Process temperature reading if successful
      if (result == 0) {
        // Outputs the data
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print(", Humidity:");
        Serial.print(humidity);
        Serial.print(", Target Temperature:");
        Serial.println(targetTemperature);

        // Update LEDs based on temperature relative to target
        if (temperature < targetTemperature - 2) {           // Below target
          digitalWrite(COLD_LED, HIGH);
          digitalWrite(OK_LED, LOW);
          digitalWrite(WARM_LED, LOW);
        } else if (temperature >= targetTemperature - 2 && temperature <= targetTemperature + 2) { // At target
          digitalWrite(COLD_LED, LOW);
          digitalWrite(OK_LED, HIGH);
          digitalWrite(WARM_LED, LOW);
        } else {                                             // Above target
          digitalWrite(COLD_LED, LOW);
          digitalWrite(OK_LED, LOW);
          digitalWrite(WARM_LED, HIGH);
        }
      } else {
        // Output error message if sensor reading fails
        Serial.println(DHT11::getErrorString(result));
      }
    }
  }
}
