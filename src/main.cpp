#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <DHT.h>
#include <BluetoothSerial.h>

#define microphone_digital_pin 25 //yellow
#define microphone_analog_pin 33 //yellow

#define infrared_digital_pin 26 //blue

#define temperature_pin 32 //green

#define accelerometer_SCL_pin 22 //white
#define accelerometer_SDA_pin 21 //white
#define accelerometer_interrupt_pin 27 //white

#define temperature_read_period 60 //how often temperature is read in seconds

#define accelerometer_threshold 1 //how strong acceleration must be to trigger interrupt
#define accelerometer_duration 5 //how long acceleration must persist to trigger interrupt

DHT temperature(temperature_pin, DHT11);

Adafruit_MPU6050 accelerometer;

hw_timer_t *timer = NULL;

const uint8_t timer_num = 0;
const uint16_t timer_divider = 80;
const uint64_t timer_alarm_value = 1000000 * temperature_read_period;

int microphone_analog_value = 0;

float temperature_value = 0;
float humidity_value = 0;
float heat_index_value = 0;

volatile bool microphone_flag = false;
volatile bool infrared_flag = false;
volatile bool temperature_flag = false;
volatile bool accelerometer_flag = false;

String device_name = "Team 9 ESP32 Sleep Monitor";

BluetoothSerial bluetooth;

void ARDUINO_ISR_ATTR microphone_ISR() {
  microphone_flag = true;
}

void ARDUINO_ISR_ATTR infrared_ISR() {
  infrared_flag = true;
}

void ARDUINO_ISR_ATTR temperature_ISR() {
  temperature_flag = true;
}

void ARDUINO_ISR_ATTR accelerometer_ISR() {
  accelerometer_flag = true;
}

void setup() {
  pinMode(microphone_digital_pin, INPUT);
  pinMode(microphone_analog_pin, INPUT);

  attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);

  pinMode(infrared_digital_pin, INPUT);

  attachInterrupt(infrared_digital_pin, infrared_ISR, RISING);

  temperature.begin();

  timer = timerBegin(timer_num, timer_divider, true);
  timerAttachInterrupt(timer, &temperature_ISR, true);
  timerAlarmWrite(timer, timer_alarm_value, true);
  timerAlarmEnable(timer);

  accelerometer.begin();

  accelerometer.setAccelerometerRange(MPU6050_RANGE_4_G);
  accelerometer.setFilterBandwidth(MPU6050_BAND_21_HZ);

  accelerometer.setMotionDetectionThreshold(accelerometer_threshold);
  accelerometer.setMotionDetectionDuration(accelerometer_duration);
  accelerometer.setMotionInterrupt(true);

  accelerometer.setInterruptPinLatch(true);
  accelerometer.setInterruptPinPolarity(false);

  pinMode(accelerometer_interrupt_pin, INPUT);

  attachInterrupt(accelerometer_interrupt_pin, accelerometer_ISR, RISING);

  bluetooth.begin(device_name);

  bluetooth.printf("ACTIVE\n");

  Serial.begin(115200);
  Serial.printf("\n\n");
}

void loop() {
  if(microphone_flag) {
    microphone_analog_value = analogRead(microphone_analog_pin);

    Serial.printf("NOISE EVENT DETECTED!\n");
    Serial.printf("ANALOG READING WAS: %d\n\n", microphone_analog_value);

    bluetooth.printf("NOISE %d\n", microphone_analog_value);

    microphone_flag = false;
  }

  if(infrared_flag) {
    Serial.printf("INFRARED MOVEMENT EVENT DETECTED!\n\n");

    bluetooth.printf("IR MOVE\n");

    infrared_flag = false;
  }

  if(temperature_flag) {
    temperature_value = temperature.readTemperature(true);
    humidity_value = temperature.readHumidity();
    heat_index_value = temperature.computeHeatIndex(temperature_value, humidity_value);

    Serial.printf("TEMPERATURE READING (IN F) IS: %f\n", temperature_value);
    Serial.printf("HUMIDITY READING (AS %%) IS: %f\n", humidity_value);
    Serial.printf("HEAT INDEX READING (IN F) IS: %f\n\n", heat_index_value);

    bluetooth.printf("TEMP %f HUMID %f INDEX %f\n", temperature_value, humidity_value, heat_index_value);

    temperature_flag = false;
  }

  if(accelerometer_flag) {
    Serial.printf("ACCELEROMETER MOVEMENT EVENT DETECTED!\n\n");

    bluetooth.printf("ACCEL MOVE\n");

    accelerometer.getMotionInterruptStatus();
    accelerometer_flag = false;
  }
}
