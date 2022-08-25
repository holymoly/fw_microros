#include <micro_ros_arduino.h>
#include "Arduino.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_raw;

rclc_executor_t executor_sub_cmd;
rclc_executor_t executor_pub_imu;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_sub_cmd;
rcl_timer_t timer_pub_imu;
Adafruit_MPU6050 mpu;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Hoverboard command
#define START_FRAME 0xABCD // [-] Start frme definition for reliable serial communication
#define RXD2 16
#define TXD2 17

typedef struct{
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial.println("error");
    delay(100);
  }
}

// ######################### Ros2 comm ###################################
void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  //Serial.print("Velocity: ");
  //Serial.print(msg->linear.x);
  //Serial.print(" : ");
  //Serial.println(msg->linear.x * 1000);
  //Serial.print("Steering");
  //Serial.print(msg->angular.z);
  //Serial.print(" : ");
  //Serial.println(msg->angular.z * 1000);

  updateHoverboard(msg->angular.z * 1000, msg->linear.x * 1000);

}

// timer based publish of IMU data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    getImuData();
    rcl_publish(&imu_publisher, &imu_raw, NULL);
  }
}

// ######################### Hoverboard comm #############################


void getImuData(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  imu_raw.linear_acceleration.x = a.acceleration.x;
  imu_raw.linear_acceleration.y = a.acceleration.y;
  imu_raw.linear_acceleration.z = a.acceleration.z;
  imu_raw.angular_velocity.x = g.gyro.x;
  imu_raw.angular_velocity.y = g.gyro.y;
  imu_raw.angular_velocity.z = g.gyro.z;
  imu_raw.header.stamp.sec = millis()/1000;
  imu_raw.header.stamp.nanosec = 0;
  imu_raw.orientation_covariance[0] = -1;
  MadgwickQuaternionUpdate(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);
}

void updateHoverboard(int16_t uSteer, int16_t uSpeed)
{
  Serial.print("Velocity: ");
  Serial.println(uSpeed);
  Serial.print("Steering");
  Serial.println(uSteer);


  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

// ################## calculate Quaternions ##############################
// adopted the calculation from https://github.com/kriswiner/MPU6050/blob/master/quaternionFilter.ino
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;

   Now = micros();
   deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
   lastUpdate = Now;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gyrox -= gbiasx;
  gyroy -= gbiasy;
  gyroz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
  qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
  qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
  qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * deltat;
  q2 += (qDot2 -(beta * hatDot2)) * deltat;
  q3 += (qDot3 -(beta * hatDot3)) * deltat;
  q4 += (qDot4 -(beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
  
  imu_raw.orientation.x = q[1]; //x
  imu_raw.orientation.y = q[2]; //y
  imu_raw.orientation.z = q[3]; //z
  imu_raw.orientation.w = q[0]; //w

}

void setup() {
  Serial.begin(115200); // USB to PC
  Serial2.begin(115200,SERIAL_8N1, RXD2, TXD2); // UART to Hoverboard

  // Try to initialize!
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("MPU6050 Found!");

  set_microros_wifi_transports("ZTLiot", "C8<y<+Et", "10.42.1.184", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));


  // create subscriber
  RCCHECK(rclc_subscription_init_default(
	&subscriber,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	"cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
	&imu_publisher,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
	"/imu/imu_raw"));

  // create timer,
  const unsigned int timer_timeout_pub = 10;
  RCCHECK(rclc_timer_init_default(
	&timer_pub_imu,
	&support,
	RCL_MS_TO_NS(timer_timeout_pub),
	timer_callback));


  imu_raw.header.frame_id.data = "MPU6050";
  imu_raw.header.frame_id.size = 10;

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub_cmd, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_cmd, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&executor_pub_imu, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub_imu, &timer_pub_imu));

  Serial.println("ready");
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor_sub_cmd, RCL_MS_TO_NS(100))); 
  RCCHECK(rclc_executor_spin_some(&executor_pub_imu, RCL_MS_TO_NS(100)));
}
