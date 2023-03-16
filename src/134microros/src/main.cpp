#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

rcl_publisher_t pot_pub;
std_msgs__msg__Int64 pot_msg;

rcl_publisher_t joy_pub;
std_msgs__msg__Int64 joy_msg;

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

rclc_executor_t executor_pub;
rclc_executor_t executor_sub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


int count_ones;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#define LED_PIN 2

#define T_sw 8
#define EM_sw 7
#define C_touch 6
#define J_dir 4

#define EM_out 5

#define SCK 13
#define MISO 12
#define MOSI 11
#define CS 10

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

int64_t readPots() {
  int64_t joints = 0;
  for(int i=0; i<5;i++) {
    joints *= 1000;
    int v = adc.readADC(i)*1000/1024;
    joints += (v > 999) ? 999 : v;
  }
  return joints;
}

int64_t readJoystick() {
  int64_t joints = 0;
  for(int i=5; i<7;i++) {
    joints *= 1000;
    int v = adc.readADC(i)*1000/1024;
    joints += (v > 999) ? 999 : v;
  }
  int button = digitalRead(9);
  if(button == 1) count_ones+=1;
  else count_ones=0;

  if(count_ones >= 20){
    count_ones = 20;
    button = 1;
  } else button=0;
  joints = joints*100000 + 
           button*10000 + 
           digitalRead(T_sw)*1000 + 
           digitalRead(EM_sw)*100 + 
           digitalRead(C_touch)*10 +
           digitalRead(J_dir);

  return joints;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&pot_pub, &pot_msg, NULL));
    pot_msg.data = readPots();
    RCSOFTCHECK(rcl_publish(&joy_pub, &joy_msg, NULL));
    joy_msg.data = readJoystick();
  }
}

void subscription_callback(const void * msgin) {
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if(msg->data > 0) digitalWrite(EM_out, HIGH);
  else digitalWrite(EM_out, LOW);
}

// void subscription_callback(const void * msgin)
// {
// 	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
// 	//printf("Received: %d\n", msg->data);
// }

void setup() {
  analogReadResolution(10);
  set_microros_transports();
  count_ones = 0;

  pinMode(LED_PIN, OUTPUT);

  pinMode(T_sw, INPUT);
  pinMode(EM_sw, INPUT);
  pinMode(C_touch, INPUT);
  
  pinMode(EM_out, OUTPUT);

  digitalWrite(LED_PIN, HIGH);  
  
  adc.begin();
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "rp2040_pots", "", &support));

  // create publisher  unsigned int num_handles_pub = 2;
  RCCHECK(rclc_publisher_init_default(
    &pot_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "pot_val"));
  
  RCCHECK(rclc_publisher_init_default(
    &joy_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "sensors"));

  RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"EM_enable"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  

  pot_msg.data = 0;
  joy_msg.data = 0;
}

void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(1)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1)));
}