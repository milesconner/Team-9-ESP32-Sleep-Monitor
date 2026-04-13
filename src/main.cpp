#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <DHT.h>
#include <BluetoothSerial.h>
#include <esp_sleep.h>

//pin configuration, some of these are never actually used in the code but are here for reference if the hardware ever needs put back together
#define microphone_digital_pin 25 //yellow wire
#define microphone_analog_pin 33 //yellow wire

#define infrared_digital_pin 26 //blue wire

#define temperature_pin 32 //green wire

#define accelerometer_SCL_pin 22 //white wire
#define accelerometer_SDA_pin 21 //white wire
#define accelerometer_interrupt_pin 27 //white wire

#define stop_button_pin 34 //red wire

#define temperature_read_period 600 //how often temperature is read in seconds (600 = every 10 mins)

#define accelerometer_threshold 1 //how strong acceleration must be to trigger accel interrupt (1 is most sensitive)
#define accelerometer_duration 1 //how long acceleration must persist to trigger accel interrupt (1 is shortest)

#define debounce_time 1000000 //in us, 1 s = 1000 ms = 1000000 us

#define buffer_size 2048 //how many events the ESP32 can hold (2048 is about 50 KB)

//the different states the state machine can take
//this machine only has one path it can take, states will always transition from top to bottom of enum
enum SystemState {
  WAIT_FOR_START, //waiting for phone to send start signal
  MONITORING,     //monitoring and logging sleep data
  WAIT_FOR_STOP,  //waiting for phone to send stop signal
  TRANSMIT_DATA,  //transmitting sleep data to phone
  SHUTDOWN        //disabling everything and going into deep sleep when finished
};

//what kind of sleep monitoring mode the user wants
enum SystemMode {
  IR_MODE,        //user wants to mount the device in the air, no reason to waste resources on accel
  ACCEL_MODE,     //user wants to place the device on bed, no reason to waste resources on ir sensor
  HYBRID_MODE     //not practical for a user, mostly just here to test all four sensors in the same run
};

//what type of sleep event an event object logs (ir events and accel events are rolled into one type)
enum EventType {
  EVENT_NOISE,
  EVENT_MOTION,
  EVENT_TEMP
};

//struct for logged events
struct Event {
  uint64_t timestamp; //when the event was detected (in microseconds since monitoring began (aka when we received the start signal))
  EventType type; //type of event

  //union between 3 different sub-structs, one for each event type. this saves space since each type is mutually exclusive
  union {
    //noise sub-struct
    struct {
      int microphone_analog_value; //magnitude of the mic reading (0-4095, in general a bigger number means a louder sound)
    } noise;                       //hovers close to some midpoint that depends on where the mic's potentiometer is, so it's always somewhere in the top half of the range

    //motion sub-struct
    struct {
      uint8_t source; //which sensor detected the motion: 0 is IR sensor, 1 is accelerometer
    } motion;         //will usually always be one or the other, but this lets us distinguish between ir and accel in hybrid mode

    //temp sub-struct
    struct {
      float temperature_value; //temperature in degress F
      float humidity_value;    //humidity as a percentage
      float heat_index_value;  //heat index in degrees F (how hot it "feels" factoring humidity in)
    } temp;
  };
};

//circular buffer to hold all the logged events
//if it ever fills up, it'll begin overwriting earliest events
struct EventBuffer {
  Event buffer[buffer_size];

  uint16_t head = 0;
  uint16_t tail = 0;

  bool overflow = false; //set to true if we run out of space
  uint16_t dropped_events = 0; //how many events had to be overwritten if overflow occurs
};

//temp sensor object
DHT temperature(temperature_pin, DHT11);

//accel object
Adafruit_MPU6050 accelerometer;

uint64_t monitor_start_time; //when we received start signal for time keeping
uint64_t next_temp_reading; //when the next temp reading should be taken

SystemState system_state; //current state of the state machine
SystemMode system_mode; //monitoring mode (whether the user wants ir mode, accel mode, or hybrid mode)
EventBuffer event_buffer; //sleep data buffer (circular buffer that holds every logged event for transmission)

uint16_t event_index = 0; //used for transmitting data with an index (chronological order of reported events)

//volatile bools (can be changed by an ISR at any time)
volatile bool microphone_flag = false; //mic event has occurred
volatile bool infrared_flag = false; //ir event has occurred
volatile bool accelerometer_flag = false; //accel event has occurred
volatile bool stop_button_flag = false; //stop button event has occurred

//non volatile bools
bool temperature_flag = false; //temp event has occurred
bool temperature_active = false; //for ignoring false microphone events during temperature readings

//for debouncing gpio events
uint64_t last_microphone_event = 0;
uint64_t last_accelerometer_event = 0;
uint64_t last_infrared_event = 0;
uint64_t last_stop_button_event = 0;

uint64_t current_microphone_event;
uint64_t current_accelerometer_event;
uint64_t current_infrared_event;
uint64_t current_stop_button_event;

//bluetooth
String device_name = "Team 9 ESP32 Sleep Monitor";
BluetoothSerial bluetooth;
String bluetooth_message = "";

//helper function to add a new event to the buffer at the head position
void push(EventBuffer& event_buffer, const Event& event) {
  event_buffer.buffer[event_buffer.head] = event;

  uint16_t next = (event_buffer.head + 1) % buffer_size;

  if(next == event_buffer.tail) {
    event_buffer.overflow = true;
    event_buffer.dropped_events++;

    event_buffer.tail = (event_buffer.tail + 1) % buffer_size;
  }

  event_buffer.head = next;
}

//helper function to retrieve the event at the tail position in the buffer
Event* pop(EventBuffer& event_buffer) {
  if(event_buffer.head == event_buffer.tail) {
    return NULL;
  }

  Event* event = &event_buffer.buffer[event_buffer.tail];

  event_buffer.tail = (event_buffer.tail + 1) % buffer_size;

  return event;
}

//GPIO ISRs
void ARDUINO_ISR_ATTR microphone_ISR() {
  if(! temperature_active) { //if temperature_active is true, this mic event is caused by temp sensor noise and isn't real
    microphone_flag = true;
    detachInterrupt(microphone_digital_pin);
  }
}

void ARDUINO_ISR_ATTR infrared_ISR() {
  infrared_flag = true;
  detachInterrupt(infrared_digital_pin);
}

void ARDUINO_ISR_ATTR accelerometer_ISR() {
  accelerometer_flag = true;
  detachInterrupt(accelerometer_interrupt_pin);
}

void ARDUINO_ISR_ATTR stop_button_ISR() {
  stop_button_flag = true;
  detachInterrupt(stop_button_pin);
}

//initialize monitoring and transition state, called after receiving start signal
void init_monitor() {
  monitor_start_time = esp_timer_get_time();
  next_temp_reading = monitor_start_time + (temperature_read_period * 1000000ULL);

  switch(system_mode) {
    case IR_MODE:
      accelerometer.enableSleep(true);

      attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);
      attachInterrupt(infrared_digital_pin, infrared_ISR, RISING);
      attachInterrupt(stop_button_pin, stop_button_ISR, RISING);

      break;

    case ACCEL_MODE:
      attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);
      attachInterrupt(accelerometer_interrupt_pin, accelerometer_ISR, RISING);
      attachInterrupt(stop_button_pin, stop_button_ISR, RISING);

      break;

    case HYBRID_MODE:
      attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);
      attachInterrupt(infrared_digital_pin, infrared_ISR, RISING);
      attachInterrupt(accelerometer_interrupt_pin, accelerometer_ISR, RISING);
      attachInterrupt(stop_button_pin, stop_button_ISR, RISING);

      break;
  }

  bluetooth.flush();
  delay(2000);
  bluetooth.end();
  delay(2000);

  Serial.printf("Transitioning to MONITORING\n");
  system_state = MONITORING;
}

//deinitialize monitoring and transition state, called after stop button is pushed on the device
void deinit_monitor() {
  switch(system_mode) {
    case IR_MODE:
      detachInterrupt(microphone_digital_pin);
      detachInterrupt(infrared_digital_pin);
      detachInterrupt(stop_button_pin);

      break;

    case ACCEL_MODE:
      detachInterrupt(microphone_digital_pin);
      detachInterrupt(accelerometer_interrupt_pin);
      detachInterrupt(stop_button_pin);

      break;

    case HYBRID_MODE:
      detachInterrupt(microphone_digital_pin);
      detachInterrupt(infrared_digital_pin);
      detachInterrupt(accelerometer_interrupt_pin);
      detachInterrupt(stop_button_pin);

      break;
  }

  accelerometer.enableSleep(true);

  delay(2000);
  bluetooth.begin();
  delay(2000);

  Serial.printf("Transitioning to WAIT_FOR_STOP\n");
  system_state = WAIT_FOR_STOP;
}

//called right before sleeping to prepare wakeup sources
void prepare_sleep() {
  Serial.flush();

  esp_sleep_enable_gpio_wakeup();

  switch(system_mode) {
    case IR_MODE:
      gpio_wakeup_enable((gpio_num_t) microphone_digital_pin, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable((gpio_num_t) infrared_digital_pin, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable((gpio_num_t) stop_button_pin, GPIO_INTR_HIGH_LEVEL);

      break;

    case ACCEL_MODE:
      gpio_wakeup_enable((gpio_num_t) microphone_digital_pin, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable((gpio_num_t) accelerometer_interrupt_pin, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable((gpio_num_t) stop_button_pin, GPIO_INTR_HIGH_LEVEL);

      break;

    case HYBRID_MODE:
      gpio_wakeup_enable((gpio_num_t) microphone_digital_pin, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable((gpio_num_t) infrared_digital_pin, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable((gpio_num_t) accelerometer_interrupt_pin, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable((gpio_num_t) stop_button_pin, GPIO_INTR_HIGH_LEVEL);

      break;
  }

  uint64_t now = esp_timer_get_time();

  if(next_temp_reading > now) {
    esp_sleep_enable_timer_wakeup(next_temp_reading - now);

  } else { //next temp reading is past due, want the ESP32 to immediately wake to take a temp reading
    esp_sleep_enable_timer_wakeup(10);
  }
}

//called right after waking to remove wakeup sources
void prepare_wake() {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  switch(system_mode) {
    case IR_MODE:
      gpio_wakeup_disable((gpio_num_t) microphone_digital_pin);
      gpio_wakeup_disable((gpio_num_t) infrared_digital_pin);
      gpio_wakeup_disable((gpio_num_t) stop_button_pin);

      break;

    case ACCEL_MODE:
      gpio_wakeup_disable((gpio_num_t) microphone_digital_pin);
      gpio_wakeup_disable((gpio_num_t) accelerometer_interrupt_pin);
      gpio_wakeup_disable((gpio_num_t) stop_button_pin);

      break;

    case HYBRID_MODE:
      gpio_wakeup_disable((gpio_num_t) microphone_digital_pin);
      gpio_wakeup_disable((gpio_num_t) infrared_digital_pin);
      gpio_wakeup_disable((gpio_num_t) accelerometer_interrupt_pin);
      gpio_wakeup_disable((gpio_num_t) stop_button_pin);

      break;
  }
}

//called repeatedly during WAIT_FOR_START state
void wait_for_start() {
  delay(10);

  char bluetooth_char;

  if(bluetooth.available()) {
    bluetooth_char = bluetooth.read();
    bluetooth_message += bluetooth_char;
  }
    
  if(bluetooth_char == '\n') {
    bluetooth_message.trim();

    Serial.printf("Received message: %s\n", bluetooth_message.c_str());

    if(bluetooth_message.equals("START: IR MODE")) {
      system_mode = IR_MODE;
      
      init_monitor();

    } else if(bluetooth_message.equals("START: ACCEL MODE")) {
      system_mode = ACCEL_MODE;
      
      init_monitor();

    } else if(bluetooth_message.equals("START: HYBRID MODE")) {
      system_mode = HYBRID_MODE;
      
      init_monitor();
    }

    bluetooth_message = "";
  }
}

//called repeatedly during MONITORING state, the device spends 99.9% of its life here
void monitoring() {
  delay(10);

  if(esp_timer_get_time() >= next_temp_reading) {
    temperature_flag = true;
  }

  if(microphone_flag) {
    noInterrupts();
    microphone_flag = false;
    interrupts();

    current_microphone_event = esp_timer_get_time();

    if(current_microphone_event - last_microphone_event > debounce_time) {
      last_microphone_event = current_microphone_event;

      Event event;

      event.timestamp = current_microphone_event - monitor_start_time;
      event.type = EVENT_NOISE;

      event.noise.microphone_analog_value = analogRead(microphone_analog_pin);

      push(event_buffer, event);

      Serial.printf("Logged mic event\n");
    }

    attachInterrupt(microphone_digital_pin, microphone_ISR, RISING);
  }

  if(infrared_flag) {
    noInterrupts();
    infrared_flag = false;
    interrupts();

    current_infrared_event = esp_timer_get_time();

    if(current_infrared_event - last_infrared_event > debounce_time) {
      last_infrared_event = current_infrared_event;

      Event event;

      event.timestamp = current_infrared_event - monitor_start_time;
      event.type = EVENT_MOTION;

      event.motion.source = 0;

      push(event_buffer, event);

      Serial.printf("Logged ir event\n");
    }

    attachInterrupt(infrared_digital_pin, infrared_ISR, RISING);
  }

  if(temperature_flag) {
    temperature_flag = false;

    uint64_t now = esp_timer_get_time();
  
    next_temp_reading = now + (temperature_read_period * 1000000ULL);

    Event event;

    event.timestamp = now - monitor_start_time;
    event.type = EVENT_TEMP;

    temperature_active = true;

    event.temp.temperature_value = temperature.readTemperature(true);
    event.temp.humidity_value = temperature.readHumidity();
    event.temp.heat_index_value = temperature.computeHeatIndex(event.temp.temperature_value, event.temp.humidity_value);
    
    temperature_active = false;

    push(event_buffer, event);

    Serial.printf("Logged temp event\n");
  }

  if(accelerometer_flag) {
    noInterrupts();
    accelerometer_flag = false;
    interrupts();

    accelerometer.getMotionInterruptStatus();

    current_accelerometer_event = esp_timer_get_time();

    if(current_accelerometer_event - last_accelerometer_event > debounce_time) {
      last_accelerometer_event = current_accelerometer_event;

      Event event;

      event.timestamp = current_accelerometer_event - monitor_start_time;
      event.type = EVENT_MOTION;

      event.motion.source = 1;

      push(event_buffer, event);

      Serial.printf("Logged accel event\n");
    }

    attachInterrupt(accelerometer_interrupt_pin, accelerometer_ISR, RISING);
  }

  if(stop_button_flag) {
    noInterrupts();
    stop_button_flag = false;
    interrupts();

    current_stop_button_event = esp_timer_get_time();

    if(current_stop_button_event - last_stop_button_event > debounce_time) {
      last_stop_button_event = current_stop_button_event;

      Serial.printf("Stop button press registered\n");

      deinit_monitor();

      return; //leave monitoring loop after deinit, otherwise we'll get stuck sleeping until next temp reading
    }
  }

  if(! (microphone_flag || infrared_flag || temperature_flag || accelerometer_flag || stop_button_flag)) {
    prepare_sleep();
    esp_light_sleep_start();
    prepare_wake();
  }
}

//called repeatedly during WAIT_FOR_STOP state
void wait_for_stop() {
  delay(10);

  char bluetooth_char;

  if(bluetooth.available()) {
    bluetooth_char = bluetooth.read();
    bluetooth_message += bluetooth_char;
  }
    
  if(bluetooth_char == '\n') {
    bluetooth_message.trim();

    Serial.printf("Received message: %s\n", bluetooth_message.c_str());

    if(bluetooth_message.equals("STOP")) {
      bluetooth.printf("---- BEGIN DATA ----\n");
      Serial.printf("Sent message: ---- BEGIN DATA ----\n");
      
      if(event_buffer.overflow) {
        bluetooth.printf("ESP32 RAN OUT OF MEMORY, FIRST %u DETECTED EVENTS WERE LOST!\n", event_buffer.dropped_events);
        Serial.printf("Sent message: ESP32 RAN OUT OF MEMORY, FIRST %u DETECTED EVENTS WERE LOST!\n", event_buffer.dropped_events);
      }

      Serial.printf("Transitioning to TRANSMIT_DATA\n");
      system_state = TRANSMIT_DATA;
    }

    bluetooth_message = "";
  }
}

//called repeatedly during TRANSMIT_DATA state
void transmit_data() {
  delay(10);

  Event* event = pop(event_buffer);

  if(event != NULL) {
    event_index++;

    uint32_t timestamp_seconds = event->timestamp / 1000000ULL;

    uint8_t hours = timestamp_seconds / 3600;
    uint8_t minutes = (timestamp_seconds % 3600) / 60;
    uint8_t seconds = (timestamp_seconds % 3600) % 60;

    switch(event->type) {
      case EVENT_NOISE:
        bluetooth.printf("EVENT %04u -- %02u:%02u:%02u NOISE EVENT (ANALOG VALUE: %d)\n", event_index, hours, minutes, seconds, event->noise.microphone_analog_value);
        Serial.printf("Sent message: EVENT %04u -- %02u:%02u:%02u NOISE EVENT (ANALOG VALUE: %d)\n", event_index, hours, minutes, seconds, event->noise.microphone_analog_value);
        break;
      
      case EVENT_MOTION:
        if(event->motion.source == 0) {
          bluetooth.printf("EVENT %04u -- %02u:%02u:%02u MOTION EVENT (INFRARED SENSOR)\n", event_index, hours, minutes, seconds);
          Serial.printf("Sent message: EVENT %04u -- %02u:%02u:%02u MOTION EVENT (INFRARED SENSOR)\n", event_index, hours, minutes, seconds);

        } else {
          bluetooth.printf("EVENT %04u -- %02u:%02u:%02u MOTION EVENT (ACCELEROMETER)\n", event_index, hours, minutes, seconds);
          Serial.printf("Sent message: EVENT %04u -- %02u:%02u:%02u MOTION EVENT (ACCELEROMETER)\n", event_index, hours, minutes, seconds);
        }

        break;

      case EVENT_TEMP:
        bluetooth.printf("EVENT %04u -- %02u:%02u:%02u TEMPERATURE / HUMIDITY / HEAT INDEX READING (%.2f F / %.2f %% / %.2f F)\n", event_index, hours, minutes, seconds, event->temp.temperature_value, event->temp.humidity_value, event->temp.heat_index_value);
        Serial.printf("Sent message: EVENT %04u -- %02u:%02u:%02u TEMPERATURE / HUMIDITY / HEAT INDEX READING (%.2f F / %.2f %% / %.2f F)\n", event_index, hours, minutes, seconds, event->temp.temperature_value, event->temp.humidity_value, event->temp.heat_index_value);
        break;
    }

  } else {
    bluetooth.printf("%u EVENTS SUCCESSFULLY TRANSMITTED!\n", event_index);
    Serial.printf("Sent message: %u EVENTS SUCCESSFULLY TRANSMITTED!\n", event_index);
    bluetooth.printf("---- END DATA ----\n");
    Serial.printf("Sent message: ---- END DATA ----\n");

    Serial.printf("Transitioning to SHUTDOWN\n");
    system_state = SHUTDOWN;
  }
}

//called repeatedly during SHUTDOWN state, but since it goes into deep sleep without any wake up sources the program effectively ends here
void shutdown() {
  bluetooth.flush();
  delay(2000);
  bluetooth.end();
  delay(2000);
  btStop();
  delay(2000);

  esp_deep_sleep_start();
}

//program entry point, configures sensors and initializes state machine
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

  delay(2000);
  bluetooth.begin(device_name);
  delay(2000);

  Serial.begin(115200);
  Serial.printf("\n\n");

  Serial.printf("Transitioning to WAIT_FOR_START\n");
  system_state = WAIT_FOR_START;
}

//super loop, will just keep calling the relevant state function depending on the current state of the state machine
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

    case TRANSMIT_DATA:
      transmit_data();
      break;

    case SHUTDOWN:
      shutdown();
      break;
  }
}
