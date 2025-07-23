#include <ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <AFMotor.h>

// Ultrasonic pins
const int trigPin = 15;
const int echoPin = 16;

// Motor setup
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

ros::NodeHandle nh;

// Ultrasonic message
std_msgs::Float32 distance_msg;
ros::Publisher distance_pub("ultrasonic_distance", &distance_msg);

void cmdVelCallback(const geometry_msgs::Twist &cmd_msg) {
  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;

  int baseSpeed = 200;
  int leftSpeed = baseSpeed * (linear - angular);
  int rightSpeed = baseSpeed * (linear + angular);

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor1.setSpeed(leftSpeed);
    motor2.setSpeed(leftSpeed);
  } else {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor1.setSpeed(abs(leftSpeed));
    motor2.setSpeed(abs(leftSpeed));
  }

  if (rightSpeed > 0) {
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    motor3.setSpeed(rightSpeed);
    motor4.setSpeed(rightSpeed);
  } else {
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    motor3.setSpeed(abs(rightSpeed));
    motor4.setSpeed(abs(rightSpeed));
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(distance_pub);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

void loop() {
  // Ultrasonic distance calculation
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance_cm = duration * 0.034 / 2;

  // Publish the distance
  distance_msg.data = 
distance_cm;
  distance_pub.publish(&distance_msg);

  nh.spinOnce();
  delay(100);
}
