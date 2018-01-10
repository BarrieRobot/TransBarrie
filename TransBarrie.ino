#include <Arduino.h>
#include <BasicStepperDriver.h>
#include <MultiDriver.h>
#include <DRV8825.h>
#include <ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt16.h>

void move_callback(const std_msgs::UInt16 &message);
void zeroRequest(const std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

/*/-- Definitions --/*/

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for hot drinks X axis motor
#define MOTOR_HOT_X_RPM 60
// Target RPM for hot drinks Y axis motor
#define MOTOR_HOT_Y_RPM 90
// Target RPM for cold drinks motor
#define MOTOR_COLD_RPM 90

// Cold drinks X direction motor
#define DIR_HOT_X 8
#define STEP_HOT_X 9

// Warm drinks Y direction motor
#define DIR_HOT_Y 6
#define STEP_HOT_Y 7

// Cold drinks motor
#define DIR_COLD 4
#define STEP_COLD 5

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 8

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
//BasicStepperDriver stepper_hot_X(MOTOR_STEPS, DIR_HOT_X, STEP_HOT_X);
DRV8825 stepper_hot_X(MOTOR_STEPS, DIR_HOT_X, STEP_HOT_X);
DRV8825 stepper_hot_Y(MOTOR_STEPS, DIR_HOT_Y, STEP_HOT_Y);
DRV8825 stepper_cold(MOTOR_STEPS, DIR_COLD, STEP_COLD);

MultiDriver controller(stepper_hot_X, stepper_hot_Y, stepper_cold);

// Robot Operating System
ros::NodeHandle nh;

ros::Subscriber<std_msgs::UInt16> movement_sub("barrie_movement", &move_callback);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> zero_service("zeroRequest", &zeroRequest);

/*/-- Code --/*/

void ROS_init() {
    // Initialise ROS node and add subscribe to topics
    nh.initNode();
    nh.subscribe(movement_sub);
    nh.advertiseService(zero_service);
}

void move_callback(const std_msgs::UInt16 &message) {
    // TODO: Do stuff
}

void zeroRequest(const std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    // TODO: Do more stuff
}

void setup() {
    // Init LED 'cause why not?
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Initiate USB serial connection used for ROS
    Serial.begin(57600);

    // Initialise all stepper motor instances
    stepper_hot_X.begin(MOTOR_HOT_X_RPM, MICROSTEPS);
    stepper_hot_Y.begin(MOTOR_HOT_Y_RPM, MICROSTEPS);
    stepper_cold.begin(MOTOR_COLD_RPM, MICROSTEPS);

    // Initialise ROS, its subscribers, publishers and services
    ROS_init();
    
    //while(!nh.connected()) nh.spinOnce();
    nh.loginfo("Arduino: Startup complete");
}

void loop() {
    // Do all necessary ROS synchronisations (at least once per 5 seconds)
    nh.spinOnce();
    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}
