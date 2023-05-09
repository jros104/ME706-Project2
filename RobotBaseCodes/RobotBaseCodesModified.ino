/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/

#include <Math.h>
#include <SoftwareSerial.h>
#include <Classes.h>
#include <Phototransistor.h>



// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

#define NO_READ_GYRO  //Uncomment if GYRO is not attached.
#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.


#define PHOTO_1_PIN A3
#define PHOTO_2_PIN A5
#define PHOTO_3_PIN A1
#define PHOTO_4_PIN A2

#define IR_1_PIN
#define IR_2_PIN
#define IR_3_PIN
#define IR_4_PIN

#define SONAR_ECHO_PIN
#define SONAR_TRIG_PIN

#define FAN_PIN


//State machine states
enum STATE {
  INITIALISING,
  FIRE_DETECTION,
  APPROACH_FIRE,
  OBSTACLE_AVOIDANCE,
  EXTINGUISH_FIRE,
  TESTING,
  STOPPED
};

//Serial Pointer

HardwareSerial *SerialCom;
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Initialise PID


// Initialise Phototransistors
Phototransistor Photo_R_Long(PHOTO_3_PIN, 0, 0, 200);
Phototransistor Photo_L_Long(PHOTO_4_PIN, 0, 0, 200);
Phototransistor Photo_R_Short(PHOTO_2_PIN, 361.09, -0.512, 0);
Phototransistor Photo_L_Short(PHOTO_1_PIN, 339.87, -0.503, 0);

// Initialise Sonar
Sonar sonar(7, 6, 10, 0, 0);

// Initialise IR
IRSensor IR_Long_L(A8, 5, 5, 90, 3239.7, -0.853, 10, 80);
IRSensor IR_Long_R(A4, 5, -5, -90, 3182.5, -0.852, 10, 80);

// Initialise Phototransistor
Phototransistor activeA(A0, 1, 1, 50); 

// Initialise 


// Kalman Filters


// Exit Conditions

// Timer
Timer timer_sensors(100);
Timer timer_cases(100);

// Lights
FireLights firelight(22, 23, 250);

float dist_sonar, dist_photo_R, dist_photo_L;


void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");
  BluetoothSerial.begin(115200);
  
  timer_sensors.start();
  timer_cases.start();
}



float prevTime = millis();

void loop(void) //main loop
{
  firelight.update();
  static STATE machine_state = INITIALISING;

  float x_cord = 1.0;
  float y_cord = 1.0;
  float angle = 1.0;

  if (timer_cases.expired()){
    switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case FIRE_DETECTION: //detect and point robot in direction of fire
      machine_state =  fire_detection();
      break;
    case APPROACH_FIRE:
      machine_state = approach_fire();
      break;
    case OBSTACLE_AVOIDANCE:
      machine_state = obstacle_avoidance();
      break;
    case EXTINGUISH_FIRE:
      machine_state = extinguish_fire();
      break;
    case TESTING: //Lipo Battery Volage OK
      machine_state =  testing();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
    };
    timer_cases.start();
  }
  


  if (timer_sensors.expired()){
    // Get distances of sensors
    Update_Sensors();
    timer_sensors.start();
  }

  delay(20);

}




void Initialise_Sensors(){
//Kalman set first estimate

}

void Update_Sensors(){
 //Kalamn update value
  dist_sonar = sonar.getDistance();
  dist_photo_R = analogRead(PHOTO_3_PIN);
  dist_photo_L = analogRead(PHOTO_4_PIN);
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");


  Initialise_Sensors();

  return FIRE_DETECTION;
}


int tolerance = 10; //tollerenace in analog values

STATE fire_detection(){


  wheel_kinematics(0, 0, 0.08);

  // check if fire is detected by both phototransistors
  if(Photo_R_Long.IsLightDetected() && Photo_L_Long.IsLightDetected()){
    // check if fire is centered between both phototransistors
    if ( abs(dist_photo_R - dist_photo_L) < tolerance){
      return APPROACH_FIRE;
    }
  }

  return FIRE_DETECTION;
}

STATE approach_fire(){

  float rotation = 0;

  if (sonar.getXDistance() <= 25)
  {
    if (Photo_R_Short.GetDistance() <= 25){
      wheel_kinematics(0, 0, 0);
      return EXTINGUISH_FIRE;
      //return OBSTACLE_AVOIDANCE;
    }else{
      return OBSTACLE_AVOIDANCE;
    }
  }else if (Photo_R_Short.GetDistance() > 40){
    float difference = (dist_photo_R - dist_photo_L);
    float error = 0 - difference;
    rotation = error * 0.001;
  }

  wheel_kinematics(4, 0, rotation);


  return APPROACH_FIRE;
}



STATE obstacle_avoidance(){
  // Check whether left or right IR reads larger
  // Move in the direction of larger reading
  // after certain time stop moving sideways
  // Drive straight for certain time
  bool goLeft = false;
  if (IR_Long_L.getDistance() >= IR_Long_R.getDistance()){
    goLeft = true;
  }

  Timer timer_sideways(1000); // Create a 5-second timer
  timer_sideways.start(); // Start the timer
  while (true) {
    wheel_kinematics(0,goLeft ? 4 : -4, 0);
    if (timer_sideways.expired()) {
      wheel_kinematics(0,0,0);
      wheel_kinematics(0, goLeft ? -4 : 4, 0);
      delay(20);
      wheel_kinematics(0,0,0);

      break;
    }
  }
  delay(100);

  Timer timer_forwards(1000); // Create a 5-second timer
  timer_forwards.start(); // Start the timer
  while (true) {
    wheel_kinematics(4,0,0);
    if (timer_forwards.expired()) {
      wheel_kinematics(-4, 0, 0);
      delay(20);
      wheel_kinematics(0,0,0);

      break;
    }
  }
  
  return FIRE_DETECTION;

}

STATE extinguish_fire(){
  return EXTINGUISH_FIRE;
}







STATE testing(){

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
  #endif

  Serial.println(Photo_R_Short.GetDistance());

  return TESTING;
}








//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to Testing STATE");
        return TESTING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}


void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}


#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
//     SerialCom->print("Lipo level:");
//     SerialCom->print(Lipo_level_cal);
//     SerialCom->print("%");
//     SerialCom->print(" : Raw Lipo:");
//     SerialCom->println(raw_lipo);
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

#ifndef NO_HC-SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
void read_serial_command()
{
}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control
