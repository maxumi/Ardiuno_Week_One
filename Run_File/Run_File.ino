#include <DHT11.h>
#include <Bounce2.h>
#include <EEPROM.h>

#define DHT_PIN 2
#define BUTTON_PIN 4
#define POT_PIN A0
#define INTERVAL_IN_MS 5

#define COLD_LED 10  // Red LED for below target
#define OK_LED 9     // White LED for at target
#define WARM_LED 8   // Blue LED for above target
#define PAUSE_LED 7  // Yellow LED for paused

DHT11 dht11(DHT_PIN);
Bounce2::Button button;

bool isPaused = false;  // Flag to control pausing
unsigned long lastReadingTime = 0;
const unsigned long readingInterval = 1000;

int targetTemperature = 22;  // Default target temperature
const int minTemp = 15;      // Minimum temperature (for potentiometer scaling)
const int maxTemp = 50;      // Maximum temperature (for potentiometer scaling)

/**
 * @brief Setup the hardware parts to correct pins and analogue and load memory
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

  EEPROM.get(sizeof(targetTemperature), isPaused);
  digitalWrite(PAUSE_LED, isPaused ? HIGH : LOW);

}

/**
 * @brief Loop controls changes made to 
 */
void loop() {
  button.update();

  // Handle button press to toggle pause
  if (button.pressed()) {
    isPaused = !isPaused;
    digitalWrite(PAUSE_LED, isPaused ? HIGH : LOW);  // Indicate paused state with yellow LED

    // Store paused state in EEPROM only if it changes
    EEPROM.put(sizeof(targetTemperature), isPaused);

    if (isPaused) {
      digitalWrite(COLD_LED, LOW);
      digitalWrite(OK_LED, LOW);
      digitalWrite(WARM_LED, LOW);
    }
  }

  // Only adjust target temperature and take readings when not paused
  if (!isPaused) {
    // Read and map potentiometer value to target temperature
    int potValue = analogRead(POT_PIN);
    targetTemperature = map(potValue, 0, 1023, minTemp, maxTemp);

    // Store the target temperature in EEPROM only changed
    int previousTargetTemperature;
    EEPROM.get(0, previousTargetTemperature);
    if (abs(previousTargetTemperature - targetTemperature) >= 1) {  // Change threshold
      EEPROM.put(0, targetTemperature);
    }

    unsigned long currentTime = millis();
    if (currentTime - lastReadingTime >= readingInterval) {
      lastReadingTime = currentTime;

      int temperature, humidity;
      int result = dht11.readTemperatureHumidity(temperature, humidity);

      if (result == 0) {
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print(", Humidity:");
        Serial.print(humidity);
        Serial.print(", Target Temperature:");
        Serial.println(targetTemperature);

        if (temperature < targetTemperature - 2) {           // Below
          digitalWrite(COLD_LED, HIGH);
          digitalWrite(OK_LED, LOW);
          digitalWrite(WARM_LED, LOW);
        } else if (temperature >= targetTemperature - 2 && temperature <= targetTemperature + 2) { // target
          digitalWrite(COLD_LED, LOW);
          digitalWrite(OK_LED, HIGH);
          digitalWrite(WARM_LED, LOW);
        } else {                                             // Above 
          digitalWrite(COLD_LED, LOW);
          digitalWrite(OK_LED, LOW);
          digitalWrite(WARM_LED, HIGH);
        }
      } else {
        // Print error message if sensor reading fails
        Serial.println(DHT11::getErrorString(result));
      }
    }
  }
}

