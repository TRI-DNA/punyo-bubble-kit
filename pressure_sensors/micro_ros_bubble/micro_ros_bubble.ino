// Punyo Soft-Bubble Sensor - Copyright 2023 Toyota Research Institute. All rights reserved.

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define INTERVAL 10 // delay to publish data in milliseconds; 10ms  => 100Hz

// Unused
#define LED_PIN 13 // A built-in LED is not available on the Qt Py 2040 but an RGB Neopixel is

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){onboard_pixels.setPixelColor(0, onboard_pixels.Color(255,0,0));onboard_pixels.show();delay(100);onboard_pixels.setPixelColor(0, onboard_pixels.Color(0,0,0));onboard_pixels.show();}}

#include <Adafruit_NeoPixel.h>
#define NUMPIXELS 1
#define PIN_NEOPIXEL_EXT 10 // Pin: QT Py MOSI
Adafruit_NeoPixel pixels(NUMPIXELS,PIN_NEOPIXEL_EXT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel onboard_pixels(NUMPIXELS,PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

#include <Wire.h>

// From SO: https://stackoverflow.com/questions/44748740/convert-byte-array-in-hex-to-char-array-or-string-type-arduino
void array_to_string(byte array[], unsigned int len, char buffer[])
{
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len*2] = '\0';
}

// Pressure Sensor
#include "Adafruit_MPRLS.h"
// Adafruit writes: You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Unique ID Support
// Get a serial number from the MC or its flash mem (for the RP2040).
#if defined(ARDUINO_ARCH_RP2040)
  #include <pico/unique_id.h> // https://raspberrypi.github.io/pico-sdk-doxygen/group__pico__unique__id.html
#elif defined(ARDUINO_ARCH_SAMD)
  #include <ArduinoUniqueID.h> // Not implemented yet for RP2040: https://github.com/ricaun/ArduinoUniqueID/issues/22
#endif

char deviceID[128];

void setupMPRLS()
{
  Serial.begin(115200);
#if defined(NEOPIXEL_POWER)
  // Adafruit writes: If this board has a power control pin, we must set it to output and high
  // in order to enable the NeoPixels. We put this in an #if defined so it can
  // be reused for other boards without compilation errors
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

#if defined(ARDUINO_ARCH_RP2040)
  // Some boards use Wire1 vs Wire0 for the I2C Stemma connector
  if (! mpr.begin( 0x18, &Wire1)) {
#else
  if (! mpr.begin()) {
#endif

    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
        delay(10);
      }
  }
  // Get a unique id, if one is available via the mc/flash
#if defined(ARDUINO_ARCH_RP2040)
  pico_get_unique_board_id_string(deviceID, 17);
#elif defined(ARDUINO_ARCH_SAMD)
  array_to_string(UniqueID, UniqueIDsize, deviceID);
#else
  sprintf(deviceID, "%s", "DEADBEEF");
#endif
  
  Serial.println("Found MPRLS sensor");
  
}

void error_loop(){
  while(1){
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    onboard_pixels.setPixelColor(0, onboard_pixels.Color(0,0,255));
    onboard_pixels.show();
    delay(500);
    onboard_pixels.setPixelColor(0, onboard_pixels.Color(0,0,0));
    onboard_pixels.show();
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    float pressure_hPa = mpr.readPressure();
    msg.data = pressure_hPa;
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Set the external neopixel
  pixels.setPixelColor(0, pixels.Color(msg->data, msg->data, msg->data)); // 
  pixels.show();

  // Set the onboard neopixel
  onboard_pixels.setPixelColor(0, onboard_pixels.Color(msg->data, msg->data, msg->data)); // 
  onboard_pixels.show();

  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void setup() {
  setupMPRLS();
  // While setting up, go to Orange-ish
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(255,255,0));
  pixels.show();
  
  onboard_pixels.begin();
  onboard_pixels.setPixelColor(0, onboard_pixels.Color(255,255,0));
  onboard_pixels.show();
  set_microros_transports();

  // No on-board LED
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);  

  // There appears to be a convention of waiting for a sec or 2; leaving out until the justification is found.
  // delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node, this time using default options... for the purposes of writing to the /rosout topic; not successful yet
  rcl_node_options_t node_options = rcl_node_get_default_options();
  // RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_options));
  
  char pressureTopicName[80], ledTopicName[80];
  sprintf(pressureTopicName, "%s%s/pressure", "bubble_", deviceID);
  sprintf(ledTopicName, "%s%s/led", "bubble_", deviceID); // sticking with 'led' for all lowercase convention in the topic names

  // create publisher: Pressure data
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    pressureTopicName));

  // create subscriber: LED brightness
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    ledTopicName));

  // create timer,
  const unsigned int timer_timeout = INTERVAL;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Go to green when good.
  pixels.setPixelColor(0, pixels.Color(0,255,0));
  pixels.show();

  // Same for the external neopixel.
  onboard_pixels.setPixelColor(0, onboard_pixels.Color(0,255,0));
  onboard_pixels.show();

  // Not working yet.
  //RCUTILS_LOG_INFO("testing %s", "testing");
  //RCUTILS_LOG_INFO_NAMED("logtest", "testing %s", "testing"); // logtest is the name of the logger

  msg.data = 0;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(INTERVAL)));
}
