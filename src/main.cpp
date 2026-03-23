#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <DHT.h>
#include <BluetoothSerial.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#define microphone_digital_pin 25 //yellow
#define microphone_analog_pin 33 //yellow

#define infrared_digital_pin 26 //blue

#define temperature_pin 32 //green

#define accelerometer_SCL_pin 22 //white
#define accelerometer_SDA_pin 21 //white
#define accelerometer_interrupt_pin 27 //white

#define temperature_read_period 60 //how often temperature is read in seconds

#define accelerometer_threshold 1 //how strong acceleration must be to trigger interrupt
#define accelerometer_duration 1 //how long acceleration must persist to trigger interrupt

#define debounce_time 500000 //in us, 100 ms = 100000 us

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) //for configuring gpio interrupt wake up from sleep

DHT temperature(temperature_pin, DHT11);

Adafruit_MPU6050 accelerometer;

hw_timer_t *timer = NULL;

const uint8_t timer_num = 0;
const uint16_t timer_divider = 80;
const uint64_t timer_alarm_value = 1000000 * temperature_read_period;

const gpio_num_t microphone_rtc_gpio = GPIO_NUM_25;
const gpio_num_t infrared_rtc_gpio = GPIO_NUM_26;
const gpio_num_t accelerometer_rtc_gpio = GPIO_NUM_27;

int microphone_analog_value = 0;

float temperature_value = 0;
float humidity_value = 0;
float heat_index_value = 0;

volatile bool microphone_flag = false;
volatile bool infrared_flag = false;
volatile bool temperature_flag = false;
volatile bool accelerometer_flag = false;

volatile uint64_t last_microphone_interrupt = 0;
volatile uint64_t last_accelerometer_interrupt = 0;
volatile uint64_t last_infrared_interrupt = 0;

volatile uint64_t current_microphone_interrupt = 0;
volatile uint64_t current_accelerometer_interrupt = 0;
volatile uint64_t current_infrared_interrupt = 0;

String device_name = "Team 9 ESP32 Sleep Monitor";

BluetoothSerial bluetooth;

void ARDUINO_ISR_ATTR microphone_ISR() {
  current_microphone_interrupt = esp_timer_get_time();

  if(current_microphone_interrupt - last_microphone_interrupt > debounce_time) {
    microphone_flag = true;
    last_microphone_interrupt = current_microphone_interrupt;
  }
}

void ARDUINO_ISR_ATTR infrared_ISR() {
  current_infrared_interrupt = esp_timer_get_time();

  if(current_infrared_interrupt - last_infrared_interrupt > debounce_time) {
    infrared_flag = true;
    last_infrared_interrupt = current_infrared_interrupt;
  }
}

void ARDUINO_ISR_ATTR temperature_ISR() {
  temperature_flag = true;
}

void ARDUINO_ISR_ATTR accelerometer_ISR() {
  current_accelerometer_interrupt = esp_timer_get_time();

  if(current_accelerometer_interrupt - last_accelerometer_interrupt > debounce_time) {
    accelerometer_flag = true;
    last_accelerometer_interrupt = current_accelerometer_interrupt;
  }
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

  accelerometer.setAccelerometerRange(MPU6050_RANGE_2_G);
  accelerometer.setGyroRange(MPU6050_RANGE_250_DEG);
  accelerometer.setFilterBandwidth(MPU6050_BAND_260_HZ);

  accelerometer.setMotionDetectionThreshold(accelerometer_threshold);
  accelerometer.setMotionDetectionDuration(accelerometer_duration);
  accelerometer.setMotionInterrupt(true);

  accelerometer.setInterruptPinLatch(false);
  accelerometer.setInterruptPinPolarity(false);

  pinMode(accelerometer_interrupt_pin, INPUT);

  attachInterrupt(accelerometer_interrupt_pin, accelerometer_ISR, RISING);

  bluetooth.begin(device_name);

  bluetooth.printf("ACTIVE\n");

  Serial.begin(115200);
  Serial.printf("\n\n");

  //rtc_gpio_deinit(microphone_rtc_gpio);
  //rtc_gpio_deinit(infrared_rtc_gpio);
  //rtc_gpio_deinit(accelerometer_rtc_gpio);

  //uint64_t bitmask = BUTTON_PIN_BITMASK(microphone_rtc_gpio) | BUTTON_PIN_BITMASK(infrared_rtc_gpio) | BUTTON_PIN_BITMASK(accelerometer_rtc_gpio);
  //esp_sleep_enable_ext1_wakeup(bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);

  //esp_sleep_enable_timer_wakeup(temperature_read_period * 1000000ULL);
}

void loop() {
  //esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  //if(wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
  //  temperature_flag = true;
  //}

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
    microphone_flag = false;
  }

  if(accelerometer_flag) {
    Serial.printf("ACCELEROMETER MOVEMENT EVENT DETECTED!\n\n");

    bluetooth.printf("ACCEL MOVE\n");

    accelerometer.getMotionInterruptStatus();
    accelerometer_flag = false;
  }

  //esp_light_sleep_start();
}
