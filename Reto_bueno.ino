#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

#include <Arduino.h>


rcl_publisher_t publisher_referencia;
rcl_publisher_t publisher_vang;
rcl_publisher_t publisher_control;
rcl_subscription_t subscriber;


std_msgs__msg__Float32 msg;
std_msgs__msg__Float32 ref_msg;
std_msgs__msg__Float32 rpm_msg;
std_msgs__msg__Float32 vang_msg;


//std_msgs__msg__Float32 dutycycle_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer_1;
rcl_timer_t timer_2;
rcl_timer_t timer_3;


#define LED_PIN_1 13
#define LED_PIN_2 12
#define PWM_OUT_D 18
#define PWM_OUT_I 19
#define ADC_PIN 34
#define ENCODER_PIN 23
#define pi 3.1416
//#define EN_DERECHA 19
//#define EN_IZQUIERDA 18

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const int freq = 5000;
const int resolution = 8;
const int channel_i= 0;
const int channel_d= 1;
float pot = 0;
float pwm = 0;
int cont = 0;
float rpm = 0;
float ref = 0;
float v_ang = 0;
float pwm_final = 0;
float pwm_ant = 0;

// controlador 
float kp = 20.40;
float ki = 12.29;

float e_ant = 0;
float e_act = 0;
float gn_pwm = 0;
float gn = 0;
float ts = 0.02;

//signal 
float duty_cycle = 0;
float signal_pot = 0;




void error_loop(){
  while(1){
    digitalWrite(LED_PIN_1, !digitalRead(LED_PIN_1));
    delay(100);
  }
}

void ADC_callback(rcl_timer_t * timer_1, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer_1 != NULL) {
    pot = analogRead(ADC_PIN);
  }
}

void encoder_callback()
{  
  cont ++; 
}

void rpm_callback(rcl_timer_t * timer_2, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer_2 != NULL) {
    ref = (signal_pot/4095.0)*(150.0);

    rpm = 600.0 * cont/(493.0*2.0);

    v_ang = (rpm * 2 * pi)/60.0;

    ref_msg.data = ref;
    RCSOFTCHECK(rcl_publish(&publisher_referencia, &ref_msg, NULL));

    if (ref >= 0){
      rpm_msg.data = rpm;
    }
    else{
      rpm_msg.data = -rpm;
    }
    RCSOFTCHECK(rcl_publish(&publisher_control, &rpm_msg, NULL));

    if (ref >= 0){
      vang_msg.data = v_ang;
    }
    else{
      vang_msg.data = -v_ang;
    }
    RCSOFTCHECK(rcl_publish(&publisher_vang, &vang_msg, NULL));


    cont = 0;
    
  }
}

void controlador_PI(rcl_timer_t * timer, int64_t last_call_time)
{ 
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {   
    //setpoint = (signal_pot/4095.0)*(149.0);
    

    if(ref >= 0.0 && ref <= 151.0){
      e_act = ref - rpm;
      gn = (kp * e_act) + ki * ts * (e_act + e_ant);
      e_ant = e_act;
      gn_pwm = (gn/2911.0)*255.0;
      pwm_final = gn_pwm + pwm_ant;   
      pwm_ant = pwm_final; 

      ledcWrite (channel_i, pwm_final);
      ledcWrite (channel_d, 0);
      digitalWrite(LED_PIN_2, !digitalRead(LED_PIN_2));

    } else if  (ref < 0.0 && ref >= -151.0){
      
      e_act = ref + rpm;
      gn = (kp * e_act) + ki * ts * (e_act + e_ant);
      e_ant = e_act;
      gn_pwm = (gn/-2911.0)*255.0; // 0 < x < 255

      pwm_final = gn_pwm + pwm_ant;
      pwm_ant = pwm_final;

      ledcWrite(channel_d,pwm_final);
      ledcWrite(channel_i, 0);
      digitalWrite(LED_PIN_2, !digitalRead(LED_PIN_2));

      
    }
    else{
      ledcWrite(channel_d, 0);
      ledcWrite(channel_i, 0);
    }

    pwm_final = 0.0;

  }

}

void subscription_callback(const void * msgin)
{  
  std_msgs__msg__Float32 * dutycycle_msg = (std_msgs__msg__Float32 *)msgin;
  duty_cycle = dutycycle_msg -> data;
  signal_pot = duty_cycle * 4095.0;
  

  
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(PWM_OUT_D, OUTPUT);
  pinMode(PWM_OUT_I, OUTPUT);
  digitalPinToInterrupt(ENCODER_PIN);
  digitalWrite(LED_PIN_1, HIGH);  

  pinMode(LED_PIN_2, OUTPUT);
  digitalWrite(LED_PIN_2, LOW);  
  
  ledcSetup(channel_i, freq, resolution);
  ledcAttachPin(PWM_OUT_I, channel_i);

  ledcSetup(channel_d, freq, resolution);
  ledcAttachPin(PWM_OUT_D, channel_d);

  attachInterrupt(ENCODER_PIN, encoder_callback, CHANGE);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "controller", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_referencia,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "reference"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_vang,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "angular_speed"));


    RCCHECK(rclc_publisher_init_default(
    &publisher_control,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "publisher_control"));


  // create timer
  const unsigned int timer_timeout_1 = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_1,
    &support,
    RCL_MS_TO_NS(timer_timeout_1),
    ADC_callback));

  const unsigned int timer_timeout_2 = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_2,
    &support,
    RCL_MS_TO_NS(timer_timeout_2),
    rpm_callback));
  
  const unsigned int timer3_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_3,
    &support,
    RCL_MS_TO_NS(timer3_timeout),
    controlador_PI)); 

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "signal"));



  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_3));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  msg.data = 0;
  
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  //ledcWrite(channel_d, 100);
}

