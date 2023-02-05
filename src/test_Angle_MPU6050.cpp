#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

void setup() {

  Serial.begin(9600);

  Serial.println("Starting");

  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  // .init sets the sensor model for us but we can override it if required.
  // Uncomment the next line to force the sensor model to the MS5837_30BA.
  //sensor.setModel(MS5837::MS5837_30BA);

  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Update pressure and temperature readings
  sensor.read();

  Serial.print("Pressure: ");
  Serial.print(sensor.pressure());
  Serial.println(" mbar");

  Serial.print("Temperature: ");
  Serial.print(sensor.temperature());
  Serial.println(" deg C");

  Serial.print("Depth: ");
  Serial.print(sensor.depth());
  Serial.println(" m");

  Serial.print("Altitude: ");
  Serial.print(sensor.altitude());
  Serial.println(" m above mean sea level");

  delay(1000);
}

// #include <Wire.h>
// #include "MS5837.h"
// #include <Servo.h>

// MS5837 sensor;

// #define pin1 3
// //right
// #define pin2 9
// //back
// #define pin3 11
// //left
// #define pin4 10

// #define ledPin 6

// #define PARSE_AMOUNT 6

// #include "MPU6050.h"
// MPU6050 mpu;

// void setup() {
//   pinMode(2, OUTPUT);
//     digitalWrite(2, HIGH);
//     Wire.begin();
  
//   // initialize digital pin LED_BUILTIN as an output.
//   pinMode(A2, OUTPUT);
//   Servo dr1, dr2, dr3, dr4;
// Servo LEDs;
// sensor.setModel(MS5837::MS5837_30BA);
//   sensor.setFluidDensity(997); 
// }

// // the loop function runs over and over again forever
// void loop() {
//   digitalWrite(A2, HIGH);   // turn the LED on (HIGH is the voltage level)
//   delay(1000);                       // wait for a second
//   digitalWrite(A2, LOW);    // turn the LED off by making the voltage LOW
//   delay(1000);                       // wait for a second
// }

// #include "MPU6050.h"
// MPU6050 mpu;


// void setup() {
    // pinMode(2, OUTPUT);
    // digitalWrite(2, HIGH);
    // Wire.begin();
//     Serial.begin(9600);
//     mpu.initialize();     // запускаем датчик
// }
// void loop() {
//   int16_t ax = mpu.getAccelerationX();  // ускорение по оси Х
//   // стандартный диапазон: +-2g
//   ax = constrain(ax, -16384, 16384);    // ограничиваем +-1g
//   float angle = ax / 16384.0;           // переводим в +-1.0
//   // и в угол через арксинус
//   if (angle < 0) angle = 90 - degrees(acos(angle));
//   else angle = degrees(acos(-angle)) - 90;

//   int16_t ay = mpu.getAccelerationY();  // ускорение по оси Х
//   // стандартный диапазон: +-2g
//   ay = constrain(ay, -16384, 16384);    // ограничиваем +-1g
//   float angle_y = ay / 16384.0;           // переводим в +-1.0
//   // и в угол через арксинус
//   if (angle_y < 0) angle_y = 90 - degrees(acos(angle_y));
//   else angle_y = degrees(acos(-angle_y)) - 90;
  
//   Serial.print("x*: ");Serial.print(angle);Serial.print("y*: ");Serial.println(angle_y);
//   delay(5);
// }