/**
 * @file tactile_sensor_array.ino
 * @author Sudhaavan Kumaran
 * @brief Unified firmware for Velostat-based tactile sensors.
 * @details This single file supports both a 1x4 sensor array and a single sensor.
 *          Change the SENSOR_MODE definition below to switch between configurations.
 *
 * @instructions
 * 1. Set SENSOR_MODE to SENSOR_MODE_ARRAY for the 4-sensor setup.
 * 2. Set SENSOR_MODE to SENSOR_MODE_SINGLE for the single-sensor setup.
 */

// --- Configuration ---
// Define the operating modes
#define SENSOR_MODE_SINGLE 1
#define SENSOR_MODE_ARRAY  2

// *** SELECT YOUR MODE HERE ***
// Change this line to switch between the two sensor setups.
#define SENSOR_MODE SENSOR_MODE_ARRAY


// --- Hardware & Variable Definitions based on Mode ---
#if SENSOR_MODE == SENSOR_MODE_ARRAY
  // For the 1x4 Velostat Tactile Sensor Array
  const int taxelPins[4] = {A0, A1, A2, A3};
  int taxelValues[4];

#elif SENSOR_MODE == SENSOR_MODE_SINGLE
  // For the single Velostat Tactile Sensor
  const int taxelPin = A0;
  int taxelValue;

#endif


// --- Main Program ---
void setup() {
  // This runs once for both modes
  Serial.begin(115200);
}

void loop() {
  // The correct code block will be executed based on the selected mode
#if SENSOR_MODE == SENSOR_MODE_ARRAY
  // --- Array Mode Logic ---
  // Read the analog value from each taxel
  for (int i = 0; i < 4; i++) {
    taxelValues[i] = analogRead(taxelPins[i]);
  }

  // Print values in a fixed, comma-separated order
  Serial.print(taxelValues[0]); Serial.print(",");
  Serial.print(taxelValues[1]); Serial.print(",");
  Serial.print(taxelValues[2]); Serial.print(",");
  Serial.print(taxelValues[3]); Serial.println();

#elif SENSOR_MODE == SENSOR_MODE_SINGLE
  // --- Single Sensor Mode Logic ---
  // Read the analog value from the connected pin
  taxelValue = analogRead(taxelPin);

  // Print the raw sensor value
  Serial.println(taxelValue);

#endif

  // Common delay for both modes to control the sampling rate
  delay(50); // Adjust as needed
}
