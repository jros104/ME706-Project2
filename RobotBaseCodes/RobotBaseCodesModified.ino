#include <Math.h>
#include <SoftwareSerial.h>
#include <Classes.h>


//State machine states
enum STATE {
  INITIALISING,
  FIRE_DETECTION,
  FIRE_ALIGNMENT,
  APPROACH_FIRE,
  OBSTIK,
  EXTINGUISH_FIRE,
  TESTING,
  STOPPED
};

//Serial Pointer

HardwareSerial *SerialCom;
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Initialise PID
PIDController fire_detect_PID(0.001, 0, 0.001);
PIDController fire_approach_PID(0.001, 0, 0);

// Initialise Phototransistors
Phototransistor Photo_R_Long(PHOTO_R_LONG_PIN, 0, 0, 200);
Phototransistor Photo_L_Long(PHOTO_L_LONG_PIN, 0, 0, 200);
Phototransistor Photo_R_Short(PHOTO_R_SHORT_PIN, 361.09, -0.512, 0);
Phototransistor Photo_L_Short(PHOTO_L_SHORT_PIN, 339.87, -0.503, 0);

// Initialise Sonar
Sonar sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN, 10, 0, 0);

// Initialise IR
IRSensor IR_L_Long(IR_L_LONG_PIN, 5, 5, 90, 3239.7, -0.853, 10, 80);
IRSensor IR_R_Long(IR_R_LONG_PIN, 5, -5, -90, 3182.5, -0.852, 10, 80);
IRSensor IR_L_Short(IR_L_SHORT_PIN, 8.5, -5, -90, 2095.2, -0.944, 4, 30);
IRSensor IR_R_Short(IR_R_SHORT_PIN, 8.5, -5, -90, 2095.2, -0.944, 4, 30);

// Initialise Fan
Fan fan(FAN_PIN);


// Kalman Filters
Kalman Kalman_Photo_L_Long(0.1, 1, 0, 0);
Kalman Kalman_Photo_R_Long(0.1, 1, 0, 0);
Kalman Kalman_Photo_L_Short(1, 1, 0, 0);
Kalman Kalman_Photo_R_Short(1, 1, 0, 0);

Kalman Kalman_IR_L_Long(1, 10, 80, 0);
Kalman Kalman_IR_R_Long(1, 10, 80, 0);
Kalman Kalman_IR_L_Short(1, 1, 30, 0);
Kalman Kalman_IR_R_Short(1, 1, 30, 0);


// Exit Conditions

// Timer
Timer timer_sensors(100);
Timer timer_cases(100);

// Lights
FireLights firelight(22, 23, 250);

float dist_sonar, value_photo_R_long, value_photo_L_long, dist_photo_R_short, dist_photo_L_short,
      dist_IR_R_long, dist_IR_L_long, dist_IR_R_short, dist_IR_L_short;
int fires_extinguished = 0;

//   ___ ___ _____ _   _ ___ 
//  / __| __|_   _| | | | _ \
//  \__ \ _|  | | | |_| |  _/
//  |___/___| |_|  \___/|_|  

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

//   _    ___   ___  ___ 
//  | |  / _ \ / _ \| _ \
//  | |_| (_) | (_) |  _/
//  |____\___/ \___/|_|  
                      

void loop(void) //main loop
{
  firelight.update();
  static STATE machine_state = INITIALISING;

  if (timer_cases.expired()){
    switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case FIRE_ALIGNMENT:
      machine_state = fire_alignment();
      break;
    case FIRE_DETECTION: //detect and point robot in direction of fire
      machine_state =  fire_detection();
      break;
    case APPROACH_FIRE:
      machine_state = approach_fire();
      break;
    case OBSTIK:
      machine_state = obstik();
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



//   ___ _   _ _  _  ___ _____ ___ ___  _  _ ___ 
//  | __| | | | \| |/ __|_   _|_ _/ _ \| \| / __|
//  | _|| |_| | .` | (__  | |  | | (_) | .` \__ \
//  |_|  \___/|_|\_|\___| |_| |___\___/|_|\_|___/
                                              

void Initialise_Sensors(){
//Kalman set first estimate

}

void Update_Sensors(){
 //Kalamn update value
  dist_sonar = sonar.getDistance();

  value_photo_R_long = Kalman_Photo_R_Long.updateEstimate(Photo_R_Long.getAnalog());
  value_photo_L_long = Kalman_Photo_L_Long.updateEstimate(Photo_L_Long.getAnalog());
  dist_photo_R_short = Kalman_Photo_L_Short.updateEstimate(Photo_R_Short.getDistance());
  dist_photo_L_short = Kalman_Photo_L_Short.updateEstimate(Photo_L_Short.getDistance());

  dist_IR_R_long = Kalman_IR_R_Long.updateEstimate(IR_R_Long.getDistance());
  dist_IR_L_long = Kalman_IR_L_Long.updateEstimate(IR_L_Long.getDistance());
  dist_IR_R_short = Kalman_IR_R_Short.updateEstimate(IR_R_Short.getDistance());
  dist_IR_L_short = Kalman_IR_L_Short.updateEstimate(IR_L_Short.getDistance());


}

void MoveForTime(float x_speed, float y_speed, float rot_speed, float time, bool use_breaking){
  Timer timer(time);
  timer.start(); // Start the timer
  while (true) {
    wheel_kinematics(x_speed,y_speed,rot_speed);
    if (timer.expired()) {
      if (use_breaking){
        wheel_kinematics(-x_speed, -y_speed, -rot_speed);
        delay(20);
      }
      wheel_kinematics(0,0,0);
      break;
    }
  }
}


//   ___ _  _ ___ _____ ___   _   _    ___ ___ ___ _  _  ___ 
//  |_ _| \| |_ _|_   _|_ _| /_\ | |  |_ _/ __|_ _| \| |/ __|
//   | || .` || |  | |  | | / _ \| |__ | |\__ \| || .` | (_ |
//  |___|_|\_|___| |_| |___/_/ \_\____|___|___/___|_|\_|\___|
                                                          

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




//   ___ ___ ___ ___   ___  ___ _____ ___ ___ _____ ___ ___  _  _ 
//  | __|_ _| _ \ __| |   \| __|_   _| __/ __|_   _|_ _/ _ \| \| |
//  | _| | ||   / _|  | |) | _|  | | | _| (__  | |  | | (_) | .` |
//  |_| |___|_|_\___| |___/|___| |_| |___\___| |_| |___\___/|_|\_|

STATE fire_detection(){

   wheel_kinematics(0, 0, FIRE_DETECTION_ROTATION_SPEED);

  // check if fire is detected by a phototransistors
  if(Photo_R_Long.IsLightDetected() || Photo_L_Long.IsLightDetected()){
     return FIRE_ALIGNMENT;
  }
  return FIRE_DETECTION;
}

STATE fire_alignment() {

  int error = value_photo_L_long - value_photo_R_long;
  float Z = fire_detect_PID.CalculateEffort(error,0.1);
  wheel_kinematics(0, 0, Z);

  if(abs(error) < FIRE_DETECTION_TOLERANCE){
    return APPROACH_FIRE;
  }

  return FIRE_ALIGNMENT;
  
}


//     _   ___ ___ ___  ___   _   ___ _  _   ___ ___ ___ ___ 
//    /_\ | _ \ _ \ _ \/ _ \ /_\ / __| || | | __|_ _| _ \ __|
//   / _ \|  _/  _/   / (_) / _ \ (__| __ | | _| | ||   / _| 
//  /_/ \_\_| |_| |_|_\\___/_/ \_\___|_||_| |_| |___|_|_\___|
                                                          

bool goLeft = true;
bool movingSideways = true;
bool movingForwards = false;

STATE approach_fire(){

  if(dist_sonar <= OBSTIK_DIST || dist_IR_L_short <= OBSTIK_DIST || dist_IR_R_short <= OBSTIK_DIST){
    
    wheel_kinematics(-BASE_SPEED, 0, 0);
    delay(20);
    wheel_kinematics(0, 0, 0);

    if((dist_photo_R_short + dist_photo_L_short) / 2.0 <= 20) return EXTINGUISH_FIRE;

    if (dist_IR_L_short <= OBSTIK_DIST){
      if (dist_IR_R_long >= OBSTIK_MIN_SPACE) goLeft = false;
    }else if (dist_IR_R_short <= OBSTIK_DIST){
      if (dist_IR_L_long < OBSTIK_MIN_SPACE) goLeft = false;
    }else if (dist_sonar <= OBSTIK_DIST){
      if (dist_IR_L_long < OBSTIK_MIN_SPACE) goLeft = false;
    }

    return OBSTIK;
  }

  int error = value_photo_L_long - value_photo_R_long;
  float Z = fire_detect_PID.CalculateEffort(error,0.1);

  wheel_kinematics(BASE_SPEED, 0, Z);

  return APPROACH_FIRE;
}

//    ___  ___ ___ _____ ___ _  __
//   / _ \| _ ) __|_   _|_ _| |/ /
//  | (_) | _ \__ \ | |  | || ' < 
//   \___/|___/___/ |_| |___|_|\_\
                                                                                   

STATE obstik(){

  if (movingSideways){
    wheel_kinematics(0, goLeft ? AVOID_SPEED : -AVOID_SPEED, 0);

    if (dist_IR_R_long >= OBSTIK_CLEAR && dist_IR_L_long >= OBSTIK_CLEAR && dist_sonar >= OBSTIK_CLEAR){
      MoveForTime(0, goLeft ? AVOID_SPEED : -AVOID_SPEED, 0, 100, true);
      movingSideways = false;
    }

  }
  else{
    MoveForTime(AVOID_SPEED, 0, 0, AVOID_DELAY, true);
    return FIRE_DETECTION;

  }
  
  return OBSTIK;

}


//   _____  _______ ___ _  _  ___ _   _ ___ ___ _  _   ___ ___ ___ ___ 
//  | __\ \/ /_   _|_ _| \| |/ __| | | |_ _/ __| || | | __|_ _| _ \ __|
//  | _| >  <  | |  | || .` | (_ | |_| || |\__ \ __ | | _| | ||   / _| 
//  |___/_/\_\ |_| |___|_|\_|\___|\___/|___|___/_||_| |_| |___|_|_\___|

STATE extinguish_fire(){
  fan.Toggle(ON);
  Timer timer_fan(10000);
  while (true){
    bool fire_detected = Photo_R_Short.IsLightDetected() || Photo_L_Short.IsLightDetected();
    if (timer_fan.expired() || !fire_detected){
      fan.Toggle(OFF);
      break;
    }
  }

  fires_extinguished++;

  if (fires_extinguished == 2) return TESTING;

  return FIRE_DETECTION;
}



//   _____ ___ ___ _____ ___ _  _  ___ 
//  |_   _| __/ __|_   _|_ _| \| |/ __|
//    | | | _|\__ \ | |  | || .` | (_ |
//    |_| |___|___/ |_| |___|_|\_|\___|
                                  

STATE testing(){

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
  #endif
  float rotation = 0;
  float difference = (value_photo_R_long - value_photo_L_long);
  float error = 0 - difference;
  rotation = error * 0.001;

  rotation = saturate(rotation, 0.1);

  wheel_kinematics(0, 0, 0);

  Serial.println(error);

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