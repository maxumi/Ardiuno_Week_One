#include <DHT11.h>
#include <Bounce2.h>

#define DHT_PIN 2
#define BUTTON_PIN 4
#define INTERVAL_IN_MS 5

#define COLD_LED 10 
#define OK_LED 9
#define WARM_LED 8      

DHT11 dht11(DHT_PIN);
bool ledState = LOW;
bool isPaused = false;  // Flag to control pausing

Bounce2::Button button;

unsigned long lastReadingTime = 0; // To store the last time a reading was taken
const unsigned long readingInterval = 1000; // Interval for sensor readings (in milliseconds)

void setup() {
  pinMode(COLD_LED, OUTPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(WARM_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.begin(9600); // For debugging

  button.attach(BUTTON_PIN, INPUT_PULLUP);  // Attach button to BUTTON_PIN with INPUT_PULLUP
  button.interval(INTERVAL_IN_MS);
  button.setPressedState(LOW);  // Button is pressed when state is LOW
}

void loop() {
  button.update();

  if (button.pressed()) { // Check if the button was pressed
    isPaused = !isPaused; // Toggle the paused state
    ledState = LOW;       // Reset LED state when pausing

    if (isPaused) {
      digitalWrite(COLD_LED, LOW);
      digitalWrite(OK_LED, LOW);
      digitalWrite(WARM_LED, LOW);
    }
  }

  if (!isPaused) {
    unsigned long currentTime = millis();
    if (currentTime - lastReadingTime >= readingInterval) {
      lastReadingTime = currentTime;

      int temperature, humidity;
      int result = dht11.readTemperatureHumidity(temperature, humidity);

      if (result == 0) {
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print(",Humidity:");
        Serial.println(humidity);

        if (temperature < 18) {               // Cold
          digitalWrite(COLD_LED, HIGH);
          digitalWrite(OK_LED, LOW);
          digitalWrite(WARM_LED, LOW);
        } else if (temperature >= 18 && temperature <= 25) { // Okay
          digitalWrite(COLD_LED, LOW);
          digitalWrite(OK_LED, HIGH);
          digitalWrite(WARM_LED, LOW);
        } else {                               // Warm
          digitalWrite(COLD_LED, LOW);
          digitalWrite(OK_LED, LOW);
          digitalWrite(WARM_LED, HIGH);
        }
      } else {
        Serial.println(DHT11::getErrorString(result));
      }
    }
  }
}
