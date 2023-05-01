// Define the pin number for the IR sensor
int sensorPin = A0;

const int numReadings = 100;   // Number of readings to take
int readings[numReadings];    // Array to store the readings
float index = 0;                // Index of the current reading
float total = 0;                // Total of all the readings

// Define the minimum and maximum sensor readings
int sensorMin = 1023;
int sensorMax = 0;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  int time = millis();
  // Read the sensor value
  int sensorValue = analogRead(sensorPin);

  // Check if the sensor value is a new minimum or maximum
  if (sensorValue < sensorMin) {
    sensorMin = sensorValue;
  }
  if (sensorValue > sensorMax) {
    sensorMax = sensorValue;
  }

  // Read the analog input pin
  int reading = analogRead(sensorPin);
  // Subtract the oldest reading from the total
 // total = total - readings[index];
  // Add the new reading to the total
  total = total + reading;
  // Store the new reading in the array
  //readings[index] = reading;
  // Move to the next position in the array
  index = (index + 1); //% numReadings;
  // Calculate the average of the readings
  int average = total / index;
  // Print the average to the serial monitor
  Serial.println(average);

  // Print the sensor value, minimum, and maximum to the serial monitor
  //Serial.print("Sensor value: ");
 
  //Serial.println(sensorValue);
  //Serial.print(" | Minimum: ");
  //Serial.print(sensorMin);
  //Serial.print(" | Maximum: ");
  //Serial.println(sensorMax);

  // Delay for a short period of time
  delay(50);
  if (index > numReadings){
    Serial.println("done");
   delay(1000);
  }
}