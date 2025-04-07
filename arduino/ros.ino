#include <ros.h>
#include <std_msgs/Int64.h>  // Changed from Int16 to Int64
#include <std_msgs/String.h>
#include <AccelStepper.h>

ros::NodeHandle nh_;
std_msgs::Int64 extender_message;
std_msgs::Int64 lift_message;
AccelStepper lift(AccelStepper::DRIVER, 5, 6);
AccelStepper extender(AccelStepper::DRIVER, 7, 8);

void controlLift(const std_msgs::Int64& msg) {
  lift.moveTo(msg.data);
  nh_.loginfo("[-] Lift moved to " + msg.data);
}

void controlExtender(const std_msgs::Int64& msg) {
  extender.moveTo(msg.data);
  nh_.loginfo("[-] Extender moved to " + msg.data);
}

ros::Subscriber<std_msgs::Int64> control_lift_("/control_lift", &controlLift);
ros::Subscriber<std_msgs::Int64> control_extender_("/control_extender", &controlExtender);
ros::Publisher extender_publisher("/extender_height", &extender_message);
ros::Publisher lift_publisher("/lift_height", &lift_message);

void setup() {
  Serial.begin(115200);
  Serial.println("[-] bSAT-ROS: Starting");
  Serial.println("[!] Configuring lift parameters.");
  lift.setMaxSpeed(5000.0);   // set the maximum speed
  lift.setAcceleration(500); // set acceleration
  lift.setSpeed(1000);         // set initial speed
  lift.setCurrentPosition(0); // set position
  Serial.println("[!] Lift parameters configured.");
  Serial.println("[!] Configuring extender parameters.");
  extender.setMaxSpeed(5000.0);   // set the maximum speed
  extender.setAcceleration(500); // set acceleration
  extender.setSpeed(1000);         // set initial speed
  extender.setCurrentPosition(0); // set position
  Serial.println("[!] Extender parameters configured.");
  Serial.println("[!] bSAT-ROS: Initializing node.");
  nh_.initNode();
  Serial.println("[!] bSAT-ROS: Node initialized.");
  Serial.println("[!] bSAT-ROS: Advertising lift publisher.");
  nh_.advertise(lift_publisher);
  Serial.println("[!] bSAT-ROS: Lift publisher advertised.");
  Serial.println("[!] bSAT-ROS: Advertising extender publisher.");
  nh_.advertise(extender_publisher);
  Serial.println("[!] bSAT-ROS: Extender publisher advertised.");
  Serial.println("[!] bSAT-ROS: Subscribing lift controller.");
  nh_.subscribe(control_lift_);
  Serial.println("[!] bSAT-ROS: Lift controller subscribed.");
  Serial.println("[!] bSAT-ROS: Subscribing extender controller.");
  nh_.subscribe(control_extender_);
  Serial.println("[!] bSAT-ROS: Extender controller subscribed.");
  Serial.println("[-] bSAT-ROS: Successfully started, road to Tokyo!");
}

void loop() {
  lift.run(); // run lift
  extender.run(); // run extender
  // publish data
  extender_message.data = extender.currentPosition();
  lift_message.data = lift.currentPosition();
  extender_publisher.publish(&extender_message);
  lift_publisher.publish(&lift_message);
  // debug data
  Serial.println("Extender: " + extender_message.data);
  Serial.println("Lift: " + extender_message.data);
  nh_.spinOnce();
  delay(30);
}
