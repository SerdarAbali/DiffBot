#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

// Pin variables for motors.
const int right_pwm_pin = 5;
const int right_dir_pin = A0;
const int left_pwm_pin = 6;
const int left_dir_pin = A1;
const bool left_fwd = false;
const bool right_fwd = true;

// Default_speed.
const int default_vel = 80;
int state_vel = default_vel;
const int max_vel = 255;

// Robot dimensions. In cm.
const float wheel_dist = 25.0;
ros::NodeHandle  nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// ROS stuff.

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

std_msgs::Int16 int_msg_right;
ros::Publisher motor_right_pub("/lidarbot/motor_right", &int_msg_right);

std_msgs::Int16 int_msg_left;
// ros::Publisher motor_left_pub("/lidarbot/motor_left", &int_msg_left);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}

void MoveStop() {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
}

void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;
  // Flipped r and l. Added steering scaler.
  float right_cmd = (-z_rotation*1.8)/2.0 + x;
  float left_cmd = 2.0*x - right_cmd;
  bool right_dir = (right_cmd>0)? right_fwd : !right_fwd;
  bool left_dir = (left_cmd>0)? left_fwd : !left_fwd;

  digitalWrite(right_dir_pin, right_dir);
  digitalWrite(left_dir_pin, left_dir);
  
  int right_write = int( default_vel * right_cmd);
  int left_write = int( default_vel * left_cmd );
 
  if (x == 0 && z_rotation == 0){
      MoveStop();
  }
  
  // Advertise the arduino command.
  int abs_left_write =  abs(left_write);
  int abs_right_write = abs(right_write);

  int_msg_right.data = right_write;
  int_msg_left.data = left_write;
  // motor_right_pub.publish(&int_msg_right);
  // motor_left_pub.publish(&int_msg_left);

  analogWrite(right_pwm_pin, abs_right_write);
  analogWrite(left_pwm_pin,  abs_left_write);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);


void setup() {

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
 

  pinMode(right_pwm_pin, OUTPUT);    // sets the digital pin 13 as output
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  // Set initial values for directions. Set both to forward.
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  pinMode(13, OUTPUT);
  // Send forward command.
  analogWrite(right_pwm_pin, default_vel);
  analogWrite(left_pwm_pin, default_vel);
  delay(500);
  MoveStop();

  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(motor_right_pub);
  //nh.advertise(motor_left_pub );

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);


}

void loop() {
  nh.spinOnce();
  delay(1);

  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    nh.spinOnce();
  }}
