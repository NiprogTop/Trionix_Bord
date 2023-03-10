#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <SerialParser.h>
#include <Servo.h>

#include <Wire.h>
#include "MS5837.h"
float depth_cal = 0;


// MS5837 sensor;
Servo dr1, dr2, dr3, dr4;
Servo LEDs;

//front
#define pin1 3
//right
#define pin2 9
//back
#define pin3 11
//left
#define pin4 10

#define ledPin 6

#define PARSE_AMOUNT 6

SerialParser parser(PARSE_AMOUNT);

int16_t dr1_val, dr2_val, dr3_val, dr4_val;
int16_t dr1_val_new, dr2_val_new, dr3_val_new, dr4_val_new;
int intData = 0;

int ledValue = 0;

int timr = 0;

int y_angle = 0;


#include "MPU6050.h"
MPU6050 mpu;

float angle_pitch(){
  if (timr > 300){
    timr = 0;
    int16_t ay = mpu.getAccelerationY();  // ускорение по оси y
    // стандартный диапазон: +-2g
    ay = constrain(ay, -16384, 16384);    // ограничиваем +-1g
    float angle_y = ay / 16384.0;           // переводим в +-1.0
    // и в угол через арксинус
    if (angle_y < 0) angle_y = 90 - degrees(acos(angle_y));
    else angle_y = degrees(acos(-angle_y)) - 90;
    y_angle = angle_y;
    return (angle_y);
  } else{
    return(y_angle);
  }
}

// void updateDepth(){
//     sensor.read();
// }

void initMotors(){
  dr1.attach(3);
  dr1.writeMicroseconds(1500);
  dr2.attach(9);
  dr2.writeMicroseconds(1500);
  dr3.attach(11);
  dr3.writeMicroseconds(1500);
  dr4.attach(10);
  dr4.writeMicroseconds(1500);
  // delay(7000);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);
  delay(7000);
  digitalWrite(ledPin, 0);
}

void setMotors()
{
    int *intData = parser.getData();
    dr1_val_new = map(intData[1], -100, 100, 1200, 1800);
    dr2_val_new = map(intData[2], -100, 100, 1200, 1800);
    dr3_val_new = map(intData[3], -100, 100, 1200, 1800);
    dr4_val_new = map(intData[4], -100, 100, 1200, 1800);

    if (intData[5] != ledValue) {
        ledValue = intData[5];
        analogWrite(ledPin, ledValue);
    }

        dr1.writeMicroseconds(dr1_val_new);

        dr2.writeMicroseconds(dr2_val_new);

        dr3.writeMicroseconds(dr3_val_new);

        dr4.writeMicroseconds(dr4_val_new);
}



void printData()
{
  // updateDepth(); // update pressure sensor
        String answer = "#3 "
                    // pitch
                    + String(angle_pitch()) + " " + 0 + " " + 0
                    // roll
                    // + String(mpu.getRoll()) + " "
                    // depth
                    // + String(sensor.depth() ) + ";"
                    // temp
                    // + String(sensor.temperature()) + ";";
                    // end
                    + " " + ";";

        Serial.println(answer);
    
}

int *intData;

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);  // I2C activation
  Wire.begin();
  Serial.begin(115200);  

  initMotors();

  // analogWrite(ledPin, 120);
  mpu.initialize();

  // while (!sensor.init()) {
  //   delay(1000);
  // }
  // sensor.init();

  // sensor.setModel(MS5837::MS5837_30BA);
  // sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  // depth_cal = sensor.depth(); //калибровка глубины в самом начале работы

  Serial.println("ready!");

}

void loop() {
  parser.update();
  if (parser.received()){
    setMotors();
  }
  
  printData();
  delay(5);
  timr++;

}

