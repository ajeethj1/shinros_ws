#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.

#define Enc_Front_Left_A 18  // Encoder A pin Front Left wheel
#define Enc_Front_Right_A 19 // Encoder A pin Front Right wheel
#define Enc_Back_Left_A 20   // Encoder A pin Back Left wheel
#define Enc_Back_Right_A 21  // Encoder A pin Back Right wheel

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.

#define Enc_Front_Left_B 14  // Encoder B pin Front Left wheel
#define Enc_Front_Right_B 15 // Encoder B pin Front Right wheel
#define Enc_Back_Left_B 16   // Encoder B pin Back Left wheel
#define Enc_Back_Right_B 17  // Encoder B pin Back Right wheel
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Publisher front_right_wheel_tick_count topic publisher
std_msgs::Int16 front_right_wheel_tick_count;
ros::Publisher frontrightPub("right_front_wheel", &front_right_wheel_tick_count);

// Publisher front_left_wheel_tick_count topic publisher
std_msgs::Int16 front_left_wheel_tick_count;
ros::Publisher frontleftPub("left_front_wheel", &front_left_wheel_tick_count);

// Publisher back_right_wheel_tick_count topic publisher
std_msgs::Int16 back_right_wheel_tick_count;
ros::Publisher backrightPub("right_rear_wheel", &back_right_wheel_tick_count);

// Publisher back_left_wheel_tick_count topic publisher
std_msgs::Int16 back_left_wheel_tick_count;
ros::Publisher backleftPub("left_rear_wheel", &back_left_wheel_tick_count);


 
// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
 
// Motor A connections
const int enA = 9; // PWM Pin
const int in1 = 5; // Sleep Pin
const int in2 = 6; // Direction Pin

// Motor B connections
const int enB = 12; // PWM Pin
const int in3 = 11; // Sleep Pin
const int in4 = 13; // Direction Pin
 

 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION =300;
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.05;
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_SEPARATION = 0.41;
 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 955.41404; // Originally 2880
 
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 278;
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;

 
// Turning PWM output (0 = min, 255 = max for PWM values)
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 50; 
const int PWM_MAX = 100; 
 
// Set linear velocity and PWM variable values for each wheel
double velFrontLeftWheel = 0;
double velFrontRightWheel = 0;
double velBackLeftWheel = 0;
double velBackRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;
 
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
 
/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void front_left_wheel_tick() {
   
  // Read the value for the encoder for the front left wheel
  int val = digitalRead(Enc_Front_Left_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (front_left_wheel_tick_count.data == encoder_maximum) {
      front_left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      front_left_wheel_tick_count.data++;  
    }    
  }
  else {
    if (front_left_wheel_tick_count.data == encoder_minimum) {
      front_left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      front_left_wheel_tick_count.data--;  
    }  
  }
}
void front_right_wheel_tick() {
   
  // Read the value for the encoder for the right front wheel
  int val = digitalRead(Enc_Front_Right_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (front_right_wheel_tick_count.data == encoder_maximum) {
      front_right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      front_right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (front_right_wheel_tick_count.data == encoder_minimum) {
      front_right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      front_right_wheel_tick_count.data--;  
    }  
  }
}
void back_left_wheel_tick() {
   
  // Read the value for the encoder for the left back wheel
  int val = digitalRead(Enc_Back_Left_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (back_left_wheel_tick_count.data == encoder_maximum) {
      back_left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      back_left_wheel_tick_count.data++;  
    }    
  }
  else {
    if (back_left_wheel_tick_count.data == encoder_minimum) {
      back_left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      back_left_wheel_tick_count.data--;  
    }  
  }
}



void back_right_wheel_tick()
{
   
  // Read the value for the encoder for the right back wheel
  int val = digitalRead(Enc_Back_Right_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (back_right_wheel_tick_count.data == encoder_maximum) {
      back_right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      back_right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (back_right_wheel_tick_count.data == encoder_minimum) {
      back_right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      back_right_wheel_tick_count.data--;  
    }  
  }
}

 
/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is published on the /left_front_ticks topic. 
void calc_vel_front_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
  double currentTime = (millis()/1000);
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + front_left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velFrontLeftWheel = numOfTicks/TICKS_PER_METER/(currentTime-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = front_left_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = currentTime;
 
}
// tick count message is published on the /front_right_ticks topic. 

void calc_vel_front_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
  double currentTime = millis()/1000;
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + front_right_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velFrontRightWheel = numOfTicks/TICKS_PER_METER/(currentTime-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = front_right_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = currentTime;
 
}

// tick count message is published on the /rear_left_ticks topic. 
void calc_vel_back_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
  double currentTime=millis()/1000;
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + back_left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velBackLeftWheel = (numOfTicks/TICKS_PER_METER)/(currentTime-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = back_left_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = currentTime;
 
}
// tick count message is published on the /rear_right_ticks topic. 
void calc_vel_back_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
  double currentTime=millis()/1000;
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + back_right_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velBackRightWheel = numOfTicks/TICKS_PER_METER/(currentTime-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = back_right_wheel_tick_count.data;
  // Update the timestamp
  prevTime = currentTime;
 
}

// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
   
// Calculate the PWM value given the desired velocity 
  pwmLeftReq = K_P * (cmdVel.linear.x -(cmdVel.angular.z*WHEEL_SEPARATION/2.0));
  pwmRightReq = K_P * (cmdVel.linear.x +(cmdVel.angular.z*WHEEL_SEPARATION/2.0));
 if (pwmLeftReq>0)
 {
  pwmLeftReq=pwmLeftReq+b;
 }
 else if (pwmLeftReq<0)
 {
  pwmLeftReq=pwmLeftReq-b;
 }
 else
 {}
  if (pwmRightReq>0)
 {
  pwmRightReq=pwmRightReq+b;
 }
 else if (pwmRightReq<0)
 {
  pwmRightReq=pwmRightReq-b;
 }
  else
 {}
 
  // Check if we need to turn 
  if (cmdVel.angular.z == 0.0) {
 
    // Remove any differences in wheel velocities 
    // to make sure the robot goes straight
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = ((velFrontLeftWheel+velBackLeftWheel)- (velFrontRightWheel+velBackRightWheel))/2; 
    double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;
 
    // Correct PWM values of both wheels to make the vehicle go straight
    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  }
 
  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;  
  }  
}
 
void set_pwm_values() {
 
  // These variables will hold our desired PWM values
  double pwmLeftOut = 0;
  double pwmRightOut = 0;

  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW); 
  }
 
  // Increase the required PWM if the robot is not moving 
  if (pwmLeftReq != 0 && velFrontLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velFrontRightWheel == 0) {
    pwmRightReq *= 1.5;
  }


  pwmLeftOut =abs(pwmLeftReq);
  pwmRightOut =abs(pwmRightReq);

  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;
 
  // Set the PWM value on the pins
  analogWrite(enA, pwmLeftOut); 
  analogWrite(enB, pwmRightOut); 
}
 
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
 
void setup() {
 
  // Set pin states of the encoder
  pinMode(Enc_Front_Left_A , INPUT_PULLUP);
  pinMode(Enc_Front_Left_B , INPUT);
  pinMode(Enc_Back_Left_A , INPUT_PULLUP);
  pinMode(Enc_Back_Left_B , INPUT);
  pinMode(Enc_Front_Right_A , INPUT_PULLUP);
  pinMode(Enc_Front_Right_B , INPUT);
  pinMode(Enc_Back_Right_A , INPUT_PULLUP);
  pinMode(Enc_Back_Right_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(Enc_Front_Left_A), front_left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(Enc_Back_Left_A), back_left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(Enc_Back_Right_A), back_right_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(Enc_Front_Right_A), front_right_wheel_tick, RISING);
   
  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Set the motor speed
  analogWrite(enA, 0); 
  analogWrite(enB, 0);
 
 // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(frontrightPub);
  nh.advertise(frontleftPub);
  nh.advertise(backrightPub);
  nh.advertise(backleftPub);
  nh.subscribe(subCmdVel);
}
 
void loop() {
   
  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval)
  {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    frontleftPub.publish( &front_left_wheel_tick_count );
    backleftPub.publish( &back_left_wheel_tick_count );
    frontrightPub.publish( &front_right_wheel_tick_count );
    backrightPub.publish( &back_right_wheel_tick_count );
    
    // Calculate the velocity of the right and left wheels
    calc_vel_front_right_wheel();
    calc_vel_front_left_wheel();
    calc_vel_back_right_wheel();
    calc_vel_back_left_wheel();
     
  }
   
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 0.5) 
  {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  set_pwm_values();
}
