#define JOYSTICK_1 A1
#define JOYSTICK_2 A0
#define JOYSTICK_3 A3
#define JOYSTICK_4 A2

// Smoothing constants
const float alpha = 0.2; // Adjust this value for the desired smoothing level (0.0 to 1.0)

// Variables to store raw, smoothed, and neutral joystick values
int rawValues[4];
int smoothedValues[4];
int offsetValues[4];

void calibrate() {
  // Read and store average joystick values as offsets
  int sum[4] = {0, 0, 0, 0};
  int samples = 200; // Increase the number of samples for calibration

  for (int i = 0; i < samples; i++) {
    sum[0] += analogRead(JOYSTICK_1);
    sum[1] += analogRead(JOYSTICK_2);
    sum[2] += analogRead(JOYSTICK_3);
    sum[3] += analogRead(JOYSTICK_4);
    delay(10);
  }

  offsetValues[0] = sum[0] / samples;
  offsetValues[1] = sum[1] / samples;
  offsetValues[2] = sum[2] / samples;
  offsetValues[3] = sum[3] / samples;

  Serial.println("Calibration complete!");
}

void setup() {
  Serial.begin(9600);

  // Perform calibration during startup
  calibrate();
}

void loop() {
  // Read raw joystick values
  rawValues[0] = analogRead(JOYSTICK_1);
  rawValues[1] = analogRead(JOYSTICK_2);
  rawValues[2] = analogRead(JOYSTICK_3);
  rawValues[3] = analogRead(JOYSTICK_4);

  // Apply smoothing
  for (int i = 0; i < 4; i++) {
    smoothedValues[i] = alpha * (rawValues[i] - offsetValues[i]) + (1 - alpha) * smoothedValues[i];
  }

  // Map the smoothed values to the range 1 to 100
  int mappedValues[4];
  for (int i = 0; i < 4; i++) {
    mappedValues[i] = map(smoothedValues[i], 0, 1023, 1, 100);
    if (mappedValues[i] == -1 )
    {
      mappedValues[i] = 0;
    }
  }

  // Print mapped values
  Serial.print("Mapped Values: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(mappedValues[i] -14);
    Serial.print("\t");
  }
  Serial.println();

  delay(50); // Adjust the delay as needed for your application
}
