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

#define temperature_read_period 300 //how often temperature is read in seconds

#define accelerometer_threshold 1 //how strong acceleration must be to trigger interrupt
#define accelerometer_duration 1 //how long acceleration must persist to trigger interrupt

#define debounce_time 1000000 //in us, 1 s = 1000 ms = 1000000 us

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) //for configuring gpio interrupt wake up from sleep

#define BUFFER_SIZE 4096 //how many events the ESP32 can hold

DHT temperature(temperature_pin, DHT11);

Adafruit_MPU6050 accelerometer;

const gpio_num_t microphone_rtc_gpio = GPIO_NUM_25;
const gpio_num_t infrared_rtc_gpio = GPIO_NUM_26;
const gpio_num_t accelerometer_rtc_gpio = GPIO_NUM_27;

volatile bool microphone_flag = false;
volatile bool infrared_flag = false;
volatile bool temperature_flag = false;
volatile bool accelerometer_flag = false;

volatile bool microphone_false_alarm = true;
volatile bool temperature_is_done = false;

volatile uint64_t last_microphone_interrupt = 0;
volatile uint64_t last_accelerometer_interrupt = 0;
volatile uint64_t last_infrared_interrupt = 0;

volatile uint64_t current_microphone_interrupt = 0;
volatile uint64_t current_accelerometer_interrupt = 0;
volatile uint64_t current_infrared_interrupt = 0;

uint64_t monitor_start_time = 0; //tracks when we receive start signal

String device_name = "Team 9 ESP32 Sleep Monitor";

BluetoothSerial bluetooth;

EventBuffer BUFFER;

enum EventType {
  EVENT_NOISE,
  EVENT_MOTION,
  EVENT_TEMP
};

struct Event {
  uint64_t timestamp;
  EventType type;

  union {
    struct {
      int microphone_analog_value;
    } noise;

    struct {
      uint8_t source; //0 is IR sensor, 1 is accelerometer
    } motion;

    struct {
      float temperature_value;
      float humidity_value;
      float heat_index_value;
    } temp;
  };
};

struct EventBuffer {
  Event buffer[BUFFER_SIZE];

  uint16_t head = 0;
  uint16_t tail = 0;
  
  bool overflow = false; //set to true if we run out of space
};

void push(EventBuffer &event_buffer, const Event &event) {
  event_buffer.buffer[event_buffer.head] = event;

  uint16_t next = (event_buffer.head + 1) % BUFFER_SIZE;

  if(next == event_buffer.tail) {
    event_buffer.overflow = true;
    event_buffer.tail = (event_buffer.tail + 1) % BUFFER_SIZE;
  }

  event_buffer.head = next;
}

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

  rtc_gpio_deinit(microphone_rtc_gpio);
  rtc_gpio_deinit(infrared_rtc_gpio);
  rtc_gpio_deinit(accelerometer_rtc_gpio);

  uint64_t bitmask = BUTTON_PIN_BITMASK(microphone_rtc_gpio) | BUTTON_PIN_BITMASK(infrared_rtc_gpio) | BUTTON_PIN_BITMASK(accelerometer_rtc_gpio);
  esp_sleep_enable_ext1_wakeup(bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);

  esp_sleep_enable_timer_wakeup(temperature_read_period * 1000000ULL);

  bluetooth.begin(device_name);

  Serial.begin(115200);
  Serial.printf("\n\n");
}

void loop() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if((! temperature_is_done) && wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    temperature_flag = true;
  }

  if(microphone_flag) {
    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_NOISE;

    event.noise.microphone_analog_value = analogRead(microphone_analog_pin);

    push(BUFFER, event);

    microphone_flag = false;
  }

  if(infrared_flag) {
    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_MOTION;

    event.motion.source = 0;

    push(BUFFER, event);

    infrared_flag = false;
  }

  if(temperature_flag) {
    if(microphone_flag) {
      microphone_false_alarm = false;
    }

    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_TEMP;

    event.temp.temperature_value = temperature.readTemperature(true);
    event.temp.humidity_value = temperature.readHumidity();
    event.temp.heat_index_value = temperature.computeHeatIndex(event.temp.temperature_value, event.temp.humidity_value);
    
    if(microphone_false_alarm) {
      microphone_flag = false;

    } else {
      microphone_false_alarm = true;
    }

    push(BUFFER, event);

    temperature_is_done = true;
    temperature_flag = false;
  }

  if(accelerometer_flag) {
    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_MOTION;

    event.motion.source = 1;

    push(BUFFER, event);

    accelerometer.getMotionInterruptStatus();
    accelerometer_flag = false;
  }

  if(! (microphone_flag || infrared_flag || accelerometer_flag)) {
    temperature_is_done = false;
    esp_light_sleep_start();
  }
}
