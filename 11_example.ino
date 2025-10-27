#include <Servo.h>


#define PIN_LED   9   
#define PIN_TRIG  12  
#define PIN_ECHO  13  
#define PIN_SERVO 10  


#define SND_VEL 346.0     
#define INTERVAL 100      
#define PULSE_DURATION 10 
#define _DIST_MIN 180.0   
#define _DIST_MAX 360.0   

#define TIMEOUT 25000     
#define SCALE (0.001 * 0.5 * SND_VEL) 

#define _EMA_ALPHA 0.3 

#define _DUTY_MIN 1000  
#define _DUTY_MAX 2000   


float dist_ema = _DIST_MIN;
unsigned long last_sampling_time = 0;
bool first_measurement = true;
bool led_state = false;

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, LOW);   // LED off initially

  // initialize servo
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_MIN); // Start at 0 degree

  // initialize serial port
  Serial.begin(57600);
  Serial.println("Ultrasonic Sensor with Servo Control - Ready");
  
  delay(500);
}

void loop() {
  float dist_raw;
  
  // wait until next sampling time
  unsigned long current_time = millis();
  if (current_time < last_sampling_time + INTERVAL) {
    return;
  }
  last_sampling_time = current_time;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Apply range filter (18-36cm)
  bool in_range = (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX);
  
  // Control LED based on range
  digitalWrite(PIN_LED, in_range ? HIGH : LOW);

  // Apply EMA filter only for in-range values
  float filtered_dist;
  if (in_range) {
    if (first_measurement) {
      dist_ema = dist_raw;
      first_measurement = false;
      filtered_dist = dist_raw;
    } else {
      dist_ema = (_EMA_ALPHA * dist_raw) + ((1 - _EMA_ALPHA) * dist_ema);
      filtered_dist = dist_ema;
    }
  } else {
    filtered_dist = 0.0; // Out of range
  }

  // Control servo based on filtered distance
  int servo_angle = calculateServoAngle(filtered_dist);
  int duty_us = map(servo_angle, 0, 180, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(duty_us);

  // Output the distance to the serial port in required format
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(servo_angle);
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
}

// Calculate servo angle based on distance (0-180 degrees for 18-36cm)
int calculateServoAngle(float distance) {
  if (distance == 0.0) {
    return 0; // Out of range - 0 degrees
  }
  
  if (distance <= _DIST_MIN) {
    return 0; // 18cm or less - 0 degrees
  } else if (distance >= _DIST_MAX) {
    return 180; // 36cm or more - 180 degrees
  } else {
    // Linear mapping: 180-360mm (18-36cm) -> 0-180 degrees
    // Example: 190mm (19cm) = (190-180) * (180/(360-180)) = 10 * 1 = 10°
    // Example: 270mm (27cm) = (270-180) * (180/180) = 90 * 1 = 90°
    // Example: 350mm (35cm) = (350-180) * 1 = 170°
    float angle = (distance - _DIST_MIN) * (180.0 / (_DIST_MAX - _DIST_MIN));
    return (int)angle;
  }
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, TIMEOUT);
  
  if (duration == 0) {
    return 0.0; // No echo received or timeout
  }
  
  float distance = duration * SCALE;
  
  // Additional validation for reasonable distances
  if (distance < 20.0 || distance > 4000.0) {
    return 0.0;
  }
  
  return distance;
}
