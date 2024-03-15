#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

rcl_publisher_t publisher_1;
rcl_publisher_t publisher_2;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;
rcl_timer_t timer_2;

#define LED_PIN 13
#define ADC 36
#define PWM 15
#define PWMchan 0
#define PWMres 8
#define PWMfreq 5000

#define ENTA
#define ENTB

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback_1(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int raw_value = analogRead(ADC); //Lectura del pot
    msg.data = raw_value; //se guarda el valor en el mensaje
    float pwmcalculado;
    pwmcalculado=raw_value/16;
    ledcWrite(PWMchan, pwmcalculado);
    RCSOFTCHECK(rcl_publish(&publisher_1, &msg, NULL)); 
  }
} 

void timer_callback_2(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int raw_value = analogRead(ADC); //lectura del pot
    float voltage = raw_value * (3.3 / 4095); //Obtencion del voltaje
    msg.data = voltage; //se guarda el valor generado
    RCSOFTCHECK(rcl_publish(&publisher_2, &msg, NULL));
  }
}

void pwmcallback(const void * msgin){
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin; 
  ledcWrite(PWMchan, msg->data);
}

void encoderCallback(last_call_time){

}

void setup() {

  
  set_microros_transports();

  //configuración LEDs
  pinMode(PWM, OUTPUT);
  ledcSetup(PWMchan, PWMfreq, PWMres);
  ledcAttachPin(PWM, PWMchan);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(5000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "raw_pot"));
    
  RCCHECK(rclc_publisher_init_default(
    &publisher_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "voltage"));

  //Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_duty_cycle"));

  // create timer,
  const unsigned int timer_timeout_1 = 10;
  const unsigned int timer_timeout_2 = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_1,
    &support,
    RCL_MS_TO_NS(timer_timeout_1),
    timer_callback_1));
    
  RCCHECK(rclc_timer_init_default(
    &timer_2,
    &support,
    RCL_MS_TO_NS(timer_timeout_2),
    timer_callback_2));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &pwmcallback, ON_NEW_DATA));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
 
