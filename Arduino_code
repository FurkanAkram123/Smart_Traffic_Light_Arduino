#include <RTClib.h>
 
RTC_DS3231 rtc;
 
// the pin that is connected to SQW used for RTC
#define CLOCK_INTERRUPT_PIN 2
 
int led_state = -1;
#define button_Pin 3
 
DateTime alarm_time;
volatile bool buttonPress = false;
bool ultrasonic_smart_change = false;
 
int ultrasonic_distance_array[4] = {100, 100, 100, 100};
int ultrasonic_avg_distance_array[4] = {30, 30, 30, 30};
int ultrasonic_low_distance_counter = 0;
 
int ultrasonic_velocity_array[4] = {0, 0, 0, 0};
unsigned long past_time_ultrasonic_measure = millis();
 
/*
                        Direction 1
*/
 
// parameters for Ultrasonic Sensor 1-- change these parameters to the pins you have the sensor physically connected to
const int trigPin1 = 4;
const int echoPin1 = 5;
int USS1_Red = 23;
int USS1_Yellow = 25;
int USS1_Green = 27;
 
// Variables for the duration and the distance for sensor 3-- change these parameters to the pins you have the sensor physically connected to
const int trigPin3 = 8;
const int echoPin3 = 9;
int USS3_Red = 39;
int USS3_Yellow = 41;
int USS3_Green = 43;
 
 
 
/*
                        Direction 2
*/
 
 
 
// parameters for Ultrasonic Sensor 2 -- change these parameters to the pins you have the sensor physically connected to
const int trigPin2 = 6;
const int echoPin2 = 7;
int USS2_Red = 31;
int USS2_Yellow = 33;
int USS2_Green = 35;
 
// Variables for the duration and the distance for sensor 4-- change these parameters to the pins you have the sensor physically connected to
const int trigPin4 = 10;
const int echoPin4 = 11;
int USS4_Red = 47;
int USS4_Yellow = 49;
int USS4_Green = 51;
 
/*#####################################################################################################
 
 
                                          Setup
 
 
  #####################################################################################################
*/
 
void setup() {
  //initialise all pins for Ultra sonic sensor 1
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(USS1_Red, OUTPUT);
  pinMode(USS1_Yellow, OUTPUT);
  pinMode(USS1_Green, OUTPUT);
 
  //initialise all pins for Ultra sonic sensor 2
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
  pinMode(USS2_Red, OUTPUT);
  pinMode(USS2_Yellow, OUTPUT);
  pinMode(USS2_Green, OUTPUT);
 
  //initialise all pins for Ultra sonic sensor 3
  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin3, INPUT); // Sets the echoPin as an Input
  pinMode(USS3_Red, OUTPUT);
  pinMode(USS3_Yellow, OUTPUT);
  pinMode(USS3_Green, OUTPUT);
 
  //initialise all pins for Ultra sonic sensor 4
  pinMode(trigPin4, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin4, INPUT); // Sets the echoPin as an Input
  pinMode(USS4_Red, OUTPUT);
  pinMode(USS4_Yellow, OUTPUT);
  pinMode(USS4_Green, OUTPUT);
 
  // Making it so, that the alarm will trigger an interrupt
  pinMode(button_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_Pin), button_press, RISING);
 
  change_all('N');
 
  Serial.begin(9600);
 
 
  // initializing the rtc
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    abort();
  }
 
  if (rtc.lostPower()) {
    // this will adjust to the date and time at compilation
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
 
  //we don't need the 32K Pin, so disable it
  rtc.disable32K();
 
  // Making it so, that the alarm will trigger an interrupt
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
 
  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
 
  // stop oscillating signals at SQW Pin
  // otherwise setAlarm1 will fail
  rtc.writeSqwPinMode(DS3231_OFF);
 
  // turn off alarm 2 (in case it isn't off already)
  // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  rtc.disableAlarm(2);
 
  // Initial Street Light Condition
 
  traffic_change('r', USS2_Red, USS2_Yellow, USS2_Green);
  traffic_change('r', USS4_Red, USS4_Yellow, USS4_Green);
  traffic_change('g', USS1_Red, USS1_Yellow, USS1_Green);
  traffic_change('g', USS3_Red, USS3_Yellow, USS3_Green);
 
 
  // schedule an alarm 20 seconds in the future
  alarm_time = rtc.now() + TimeSpan(20);
 
 
  if (!rtc.setAlarm1(
        alarm_time,
        DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
      )) {
    Serial.println("Error, alarm wasn't set!");
  } else {
    Serial.println("Alarm will happen in 20 seconds!");
  }
}
 
/*#####################################################################################################
 
 
                                     Loop
 
 
  #####################################################################################################
*/
 
void loop() {
  if (rtc.alarmFired(1)) {
    traffic_state(led_state);
    led_state = led_state * (-1);
 
    rtc.clearAlarm(1);
    Serial.println("Alarm cleared");
 
    // schedule an alarm 10 seconds in the future
    alarm_time = rtc.now() + TimeSpan(20);
    if (!rtc.setAlarm1(
          alarm_time,
          DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
        )) {
      Serial.println("Error, alarm wasn't set!");
    } else {
      Serial.println("Alarm will happen in 20 seconds!");
    }
  }
 
  else if (buttonPress || ultrasonic_smart_change) {
    int time_diff = timeTillAlarm();
 
    //  DateTime timeDiff;
    //  timeDiff = rtc.now() - TimeSpan(alarm_time.day(), alarm_time.hour(), alarm_time.minute(), alarm_time.second());
 
    if (time_diff > 5 && time_diff < 10) {
 
      if (buttonPress) {
        Serial.println("Pedestrian Button Pushed Early State Change");
      }
      else if (ultrasonic_smart_change) {
        Serial.println("Traffic Flow Based Early State Change");
      }
      buttonPress = false;
      ultrasonic_smart_change = false;
      rtc.clearAlarm(1);
      Serial.println("Alarm cleared");
 
      alarm_time = rtc.now() + TimeSpan(5);
      if (!rtc.setAlarm1(
            alarm_time,
            DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
          )) {
        Serial.println("Error, alarm wasn't set!");
      } else {
        Serial.println("Alarm will happen in 5 seconds!");
      }
      delay(250);
    }
  }
 
  // Every 0.25 Second Do This
  if ((millis() - past_time_ultrasonic_measure) > 250) {
    ultrasonic_update();
    Serial.println("******************* Ultrasonic Reading *******************");
 
    Serial.print("Distance -> 1: "); Serial.print(ultrasonic_distance_array[0]); Serial.print("cm ,\t");
    Serial.print("2: "); Serial.print(ultrasonic_distance_array[1]); Serial.print("cm,\t");
    Serial.print("3: "); Serial.print(ultrasonic_distance_array[2]); Serial.print("cm,\t");
    Serial.print("4: "); Serial.print(ultrasonic_distance_array[3]); Serial.println("cm");
 
    Serial.print("Velocity -> 1: "); Serial.print(ultrasonic_velocity_array[0]); Serial.print("cm/s ,\t");
    Serial.print("2: "); Serial.print(ultrasonic_velocity_array[1]); Serial.print("cm/s ,\t");
    Serial.print("3: "); Serial.print(ultrasonic_velocity_array[2]); Serial.print("cm/s ,\t");
    Serial.print("4: "); Serial.print(ultrasonic_velocity_array[3]); Serial.println("cm/s ");
 
    Serial.println("*******************                    *******************");
 
    past_time_ultrasonic_measure = millis();
 
  }
 
}
 
int timeTillAlarm() {
  int time_present_s = int(rtc.now().second());
  int time_alarm_s = int(alarm_time.second());
 
  int time_diff_1 = time_alarm_s - time_present_s;
  int time_diff_2 = 60 - time_diff_1;
  int time_diff = min(time_diff_1, time_diff_2);
  return time_diff;
}
 
void traffic_change(char state, int red_led, int yellow_led, int green_led) {
  switch (state) {
    // Red Light
    case 'r':
      digitalWrite(red_led, HIGH);
      digitalWrite(yellow_led, LOW);
      digitalWrite(green_led, LOW);
      break;
    // Yellow Light
    case 'y':
      digitalWrite(red_led, LOW);
      digitalWrite(yellow_led, HIGH);
      digitalWrite(green_led, LOW);
      break;
    // Green Light
    case 'g':
      digitalWrite(red_led, LOW);
      digitalWrite(yellow_led, LOW);
      digitalWrite(green_led, HIGH);
      break;
 
    // All LED OFF
    case 'N':
      digitalWrite(red_led, LOW);
      digitalWrite(yellow_led, LOW);
      digitalWrite(green_led, LOW);
      break;
 
    // ALL LED ON
    case 'A':
      digitalWrite(red_led, HIGH);
      digitalWrite(yellow_led, HIGH);
      digitalWrite(green_led, HIGH);
      break;
    default:
      break;
  }
}
 
void change_all(char state) {
  traffic_change(state, USS1_Red, USS1_Yellow, USS1_Green);
  traffic_change(state, USS2_Red, USS2_Yellow, USS2_Green);
  traffic_change(state, USS3_Red, USS3_Yellow, USS3_Green);
  traffic_change(state, USS4_Red, USS4_Yellow, USS4_Green);
}
 
void button_press() {
  Serial.println("Pedestrian Button Pressed");
  buttonPress = true;
 
}
 
void traffic_monitor() {
//  for (int i = 0; i < 4; i++) {
//    if (ultrasonic_avg_distance_array[i] < 40) {
//      ultrasonic_smart_change = true;
//      break;
//    }
//  }
 
  for (int i = 0; i < 4; i++) {
    if (ultrasonic_distance_array[i] < 40) {
      ultrasonic_low_distance_counter +=1;
      ultrasonic_smart_change = true;
    }
  }
 
  if (ultrasonic_low_distance_counter>20){
    ultrasonic_smart_change = true;
    ultrasonic_low_distance_counter = 0;
    Serial.println("Traffic Flow State Change Needed");
  }
  
}
 
void traffic_state(int state) {
  switch (state) {
    // Red Light
    case -1:
      traffic_change('y', USS1_Red, USS1_Yellow, USS1_Green);
      traffic_change('y', USS3_Red, USS3_Yellow, USS3_Green);
      delay(3000);
      traffic_change('r', USS1_Red, USS1_Yellow, USS1_Green);
      traffic_change('r', USS3_Red, USS3_Yellow, USS3_Green);
      delay(1000);
 
      for (int i = 0; i < 20; i++) {
        traffic_change('N', USS2_Red, USS2_Yellow, USS2_Green);
        traffic_change('N', USS4_Red, USS4_Yellow, USS4_Green);
        delay(100);
        traffic_change('g', USS2_Red, USS2_Yellow, USS2_Green);
        traffic_change('g', USS4_Red, USS4_Yellow, USS4_Green);
        delay(100);
      }
 
      break;
    case 1:
      traffic_change('y', USS2_Red, USS2_Yellow, USS2_Green);
      traffic_change('y', USS4_Red, USS4_Yellow, USS4_Green);
      delay(3000);
      traffic_change('r', USS2_Red, USS2_Yellow, USS2_Green);
      traffic_change('r', USS4_Red, USS4_Yellow, USS4_Green);
      delay(1000);
 
      for (int i = 0; i < 20; i++) {
        traffic_change('N', USS1_Red, USS1_Yellow, USS1_Green);
        traffic_change('N', USS3_Red, USS3_Yellow, USS3_Green);
        delay(100);
        traffic_change('g', USS1_Red, USS1_Yellow, USS1_Green);
        traffic_change('g', USS3_Red, USS3_Yellow, USS3_Green);
        delay(100);
      }
 
      break;
 
    default:
      break;
  }
 
  // Reset Parameters
  for (int i = 0; i < 4; i++) {
    ultrasonic_avg_distance_array[i] =  30;
  }
  buttonPress = false;
  ultrasonic_smart_change = false;
  ultrasonic_low_distance_counter = 0;
}
 
/*#####################################################################################################
 
 
                     Function to find the distance detected by 1st Ultra Sonic Sensor
 
 
  #####################################################################################################
*/
int calculateDistance(const int trigPin, const int echoPin) {
 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  float distance = duration * 0.034 / 2; // in cm
 
  if (distance > 200){
    distance = 200;
  }
  return distance;
}
 
/*#####################################################################################################
 
 
                     Function to find the distance for all the ultrasonic sensors
 
 
  #####################################################################################################
*/
 
void ultrasonic_update() {
  float time_delta = float(millis() - past_time_ultrasonic_measure) / 1000.0;
  int temp_dist_array[4];
 
  temp_dist_array[0] = calculateDistance(trigPin1, echoPin1);
  delay(20);
  temp_dist_array[1] = calculateDistance(trigPin2, echoPin2);
  delay(20);
  temp_dist_array[2] = calculateDistance(trigPin3, echoPin3);
  delay(100);
  temp_dist_array[3] = calculateDistance(trigPin4, echoPin4);
  
 
 
  for (int i = 0; i < 4; i++) {
    ultrasonic_velocity_array[i] = (temp_dist_array[i] - ultrasonic_distance_array[i]) / time_delta;
    if (abs(ultrasonic_velocity_array[i]) < 4){
      ultrasonic_velocity_array[i] = 0;
    }
    ultrasonic_distance_array[i] = temp_dist_array[i];
    ultrasonic_avg_distance_array[i] =  (ultrasonic_avg_distance_array[i] + ultrasonic_distance_array[i]) / 2.0;
    if (abs(ultrasonic_velocity_array[i]) > 10) {
      Serial.print("Vehicle is Traffic Light: "); Serial.print(i); Serial.println(" is over the speed limit");
    }
  }
 
}
 
/*#####################################################################################################
 
 
                     Alarm Interrupt For RTC
 
 
  #####################################################################################################
*/
void onAlarm() {
  Serial.println("Smart Pedestrian Button Traffic Timer Change");
}
