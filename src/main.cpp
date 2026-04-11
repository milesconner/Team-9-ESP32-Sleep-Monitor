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

#define stop_button_pin 14 //orange

#define temperature_read_period 300 //how often temperature is read in seconds (300 = every 5 mins)

#define accelerometer_threshold 1 //how strong acceleration must be to trigger interrupt (1 is most sensitive)
#define accelerometer_duration 1 //how long acceleration must persist to trigger interrupt (1 is shortest)

#define debounce_time 1000000 //in us, 1 s = 1000 ms = 1000000 us

#define buffer_size 4096 //how many events the ESP32 can hold (4096 is about 100 KB when full)

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) //for configuring gpio interrupt wake up from sleep

enum SystemState {
  WAIT_FOR_START,
  MONITORING,
  WAIT_FOR_STOP,
  TRANSMIT_DATA,
  SHUTDOWN
};

enum SystemMode {
  IR_MODE,
  ACCEL_MODE,
  HYBRID_MODE
};

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
  Event buffer[buffer_size];

  uint16_t head = 0;
  uint16_t tail = 0;
  
  bool overflow = false; //set to true if we run out of space
};

DHT temperature(temperature_pin, DHT11);

Adafruit_MPU6050 accelerometer;

uint64_t monitor_start_time; //when we received start signal for time keeping
SystemState system_state; //current state
SystemMode system_mode; //monitoring mode
EventBuffer event_buffer; //sleep data buffer

const gpio_num_t microphone_rtc_gpio = GPIO_NUM_25;
const gpio_num_t infrared_rtc_gpio = GPIO_NUM_26;
const gpio_num_t accelerometer_rtc_gpio = GPIO_NUM_27;
const gpio_num_t stop_button_rtc_gpio = GPIO_NUM_14;

volatile bool microphone_flag = false;
volatile bool infrared_flag = false;
volatile bool temperature_flag = false;
volatile bool accelerometer_flag = false;
volatile bool stop_button_flag = false;

volatile bool temperature_active = false;
volatile bool temperature_done = false;

volatile uint64_t last_microphone_interrupt = 0;
volatile uint64_t last_accelerometer_interrupt = 0;
volatile uint64_t last_infrared_interrupt = 0;
volatile uint64_t last_stop_button_interrupt = 0;

volatile uint64_t current_microphone_interrupt;
volatile uint64_t current_accelerometer_interrupt;
volatile uint64_t current_infrared_interrupt;
volatile uint64_t current_stop_button_interrupt;

String device_name = "Team 9 ESP32 Sleep Monitor";
BluetoothSerial bluetooth;

void push(EventBuffer &event_buffer, const Event &event) {
  event_buffer.buffer[event_buffer.head] = event;

  uint16_t next = (event_buffer.head + 1) % buffer_size;

  if(next == event_buffer.tail) {
    event_buffer.overflow = true;
    event_buffer.tail = (event_buffer.tail + 1) % buffer_size;
  }

  event_buffer.head = next;
}

void ARDUINO_ISR_ATTR microphone_ISR() {
  if(temperature_active) {
    return;
  }

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

void ARDUINO_ISR_ATTR stop_button_ISR() {
  current_stop_button_interrupt = esp_timer_get_time();

  if(current_stop_button_interrupt - last_stop_button_interrupt > debounce_time) {
    stop_button_flag = true;
    last_stop_button_interrupt = current_stop_button_interrupt;
  }
}

void init_mode() {
  monitor_start_time = esp_timer_get_time();

  bluetooth.end();
  btStop();

  switch(system_mode) {
    case IR_MODE:
      accelerometer.enableSleep(true);

      attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);
      attachInterrupt(infrared_digital_pin, infrared_ISR, RISING);
      attachInterrupt(stop_button_pin, stop_button_ISR, RISING);

      rtc_gpio_deinit(microphone_rtc_gpio);
      rtc_gpio_deinit(infrared_rtc_gpio);
      rtc_gpio_deinit(stop_button_rtc_gpio);

      uint64_t bitmask = BUTTON_PIN_BITMASK(microphone_rtc_gpio) | BUTTON_PIN_BITMASK(infrared_rtc_gpio) | BUTTON_PIN_BITMASK(stop_button_rtc_gpio);
      esp_sleep_enable_ext1_wakeup(bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);

      break;

    case ACCEL_MODE:
      attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);
      attachInterrupt(accelerometer_interrupt_pin, accelerometer_ISR, RISING);
      attachInterrupt(stop_button_pin, stop_button_ISR, RISING);

      rtc_gpio_deinit(microphone_rtc_gpio);
      rtc_gpio_deinit(accelerometer_rtc_gpio);
      rtc_gpio_deinit(stop_button_rtc_gpio);

      uint64_t bitmask = BUTTON_PIN_BITMASK(microphone_rtc_gpio) | BUTTON_PIN_BITMASK(accelerometer_rtc_gpio) | BUTTON_PIN_BITMASK(stop_button_rtc_gpio);
      esp_sleep_enable_ext1_wakeup(bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);

      break;

    case HYBRID_MODE:
      attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);
      attachInterrupt(infrared_digital_pin, infrared_ISR, RISING);
      attachInterrupt(accelerometer_interrupt_pin, accelerometer_ISR, RISING);
      attachInterrupt(stop_button_pin, stop_button_ISR, RISING);

      rtc_gpio_deinit(microphone_rtc_gpio);
      rtc_gpio_deinit(infrared_rtc_gpio);
      rtc_gpio_deinit(accelerometer_rtc_gpio);
      rtc_gpio_deinit(stop_button_rtc_gpio);

      uint64_t bitmask = BUTTON_PIN_BITMASK(microphone_rtc_gpio) | BUTTON_PIN_BITMASK(infrared_rtc_gpio) | BUTTON_PIN_BITMASK(accelerometer_rtc_gpio) | BUTTON_PIN_BITMASK(stop_button_rtc_gpio);
      esp_sleep_enable_ext1_wakeup(bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);

      break;
  }

  esp_sleep_enable_timer_wakeup(temperature_read_period * 1000000ULL);

  system_state = MONITORING;
}

void wait_for_start() {
  if(bluetooth.available()) {
    String message = bluetooth.readStringUntil('\n');
    message.trim();

    if(message.equals("START: IR MODE")) {
      system_mode = IR_MODE;
      
      init_mode();

    } else if(message.equals("START: ACCEL MODE")) {
      system_mode = ACCEL_MODE;
      
      init_mode();

    } else if(message.equals("START: HYBRID MODE")) {
      system_mode = HYBRID_MODE;
      
      init_mode();
    }
  }
}

void monitoring() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if((! temperature_done) && wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    temperature_flag = true;
  }

  if(microphone_flag) {
    noInterrupts();
    microphone_flag = false;
    interrupts();

    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_NOISE;

    event.noise.microphone_analog_value = analogRead(microphone_analog_pin);

    push(event_buffer, event);
  }

  if(infrared_flag) {
    noInterrupts();
    infrared_flag = false;
    interrupts();

    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_MOTION;

    event.motion.source = 0;

    push(event_buffer, event);
  }

  if(temperature_flag) {
    temperature_flag = false;

    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_TEMP;

    temperature_active = true;

    event.temp.temperature_value = temperature.readTemperature(true);
    event.temp.humidity_value = temperature.readHumidity();
    event.temp.heat_index_value = temperature.computeHeatIndex(event.temp.temperature_value, event.temp.humidity_value);
    
    temperature_active = false;

    push(event_buffer, event);

    temperature_done = true;
  }

  if(accelerometer_flag) {
    noInterrupts();
    accelerometer_flag = false;
    interrupts();

    accelerometer.getMotionInterruptStatus();

    Event event;

    event.timestamp = esp_timer_get_time() - monitor_start_time;
    event.type = EVENT_MOTION;

    event.motion.source = 1;

    push(event_buffer, event);
  }

  if(stop_button_flag) {
    noInterrupts();
    stop_button_flag = false;
    interrupts();

    system_state = WAIT_FOR_STOP;

    bluetooth.begin(device_name);

    return;
  }

  if(! (microphone_flag || infrared_flag || temperature_flag || accelerometer_flag || stop_button_flag)) {
    temperature_done = false;
    esp_light_sleep_start();
  }
}

void wait_for_stop() {

}

void setup() {
  pinMode(microphone_digital_pin, INPUT);
  pinMode(microphone_analog_pin, INPUT);
  pinMode(infrared_digital_pin, INPUT);
  pinMode(accelerometer_interrupt_pin, INPUT);
  pinMode(stop_button_pin, INPUT);

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

  accelerometer.enableCycle(false);

  bluetooth.begin(device_name);

  system_state = WAIT_FOR_START;
}

void loop() {
  switch(system_state) {
    case WAIT_FOR_START:
      wait_for_start();
      break;

    case MONITORING:
      monitoring();
      break;

    case WAIT_FOR_STOP:
      wait_for_stop();
      break;
  }
}
