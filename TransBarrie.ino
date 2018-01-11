#include <Arduino.h>
#include <BasicStepperDriver.h>
#include <MultiDriver.h>
#include <DRV8825.h>
#include <ros.h>
#include "std_srvs/Empty.h"
#include "std_msgs/UInt16.h"
#include "ros_lib/barrieduino/Move.h"

void move_callback(const barrieduino::Move &message);
void zeroRequest(const std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

/*/-- Definitions --/*/
#define INTERRUPT_PIN_COLD 3
#define INTERRUPT_PIN_HOTX 2
#define INTERRUPT_PIN_HOTY 18

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

/* Hot transport distances  */
#define DISTANCE_CUPDISPENSER_COFFEEMACHINE 100
#define DISTANCE_COFFEEMACHINE_XYSWITCH -200
#define DISTANCE_XYSWITCH_DIAPHRAGM 100 //vertical

/* Cold transport distances  */
#define DISTANCE_RECEIVECAN_FULLYDOWN -50
#define DISTANCE_FULLYDOWN_DIAPHRAGM 200

/* shared distance */
#define DISTANCE_DIAPHRAGM_PRESENT 50 //vertical


// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
//BasicStepperDriver stepper_hot_X(MOTOR_STEPS, DIR_HOT_X, STEP_HOT_X);

/* Hot transport distances  */
DRV8825 stepper_hot_X(MOTOR_STEPS, DIR_HOT_X, STEP_HOT_X);
DRV8825 stepper_hot_Y(MOTOR_STEPS, DIR_HOT_Y, STEP_HOT_Y);
DRV8825 stepper_cold(MOTOR_STEPS, DIR_COLD, STEP_COLD);

MultiDriver controller(stepper_hot_X, stepper_hot_Y, stepper_cold);

// Robot Operating System
ros::NodeHandle nh;
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> zero_service("zeroRequest", &zeroRequest);
ros::Subscriber<barrieduino::Move> movement_sub("barrie_movement", &move_callback);

/*/-- Code --/*/
enum class HotLocation {
  cupdispenser,
  coffeemachine,
  diaphragm,
  present
};

enum class ColdLocation {
  receiveCanHeight,
  fullyDown,
  diaphragm,
  present
};

HotLocation currentHotLoc = HotLocation::cupdispenser;
ColdLocation currentColdLoc = ColdLocation::receiveCanHeight;

void ROS_init() {
    // Initialise ROS node and add subscribe to topics
    nh.initNode();
    nh.subscribe(movement_sub);
    nh.advertiseService(zero_service);
}

void move_callback(const barrieduino::Move &message) {
    // TODO: Do stuff
    // Hot drinks lane
    if (message.lane == 1) {
        switch (message.location) {
            case 0:
                // Go to cup dispenser
                // 2 step movement
                if (currentHotLoc == HotLocation::present) {
                  // Current location must be present
                  controller.rotate(0, -(DISTANCE_XYSWITCH_DIAPHRAGM + DISTANCE_DIAPHRAGM_PRESENT), 0);
                  controller.rotate(-(DISTANCE_CUPDISPENSER_COFFEEMACHINE + DISTANCE_COFFEEMACHINE_XYSWITCH), 0, 0);
                  currentHotLoc = HotLocation::cupdispenser;
                } else {
                  nh.logerror("want dispenser but " +(int) currentHotLoc);
                }
                break;
            case 1:
                // Go to coffee machine
                if (currentHotLoc == HotLocation::cupdispenser) {
                  // Current location must be cupdispenser
                  controller.rotate(DISTANCE_CUPDISPENSER_COFFEEMACHINE, 0, 0);
                  currentHotLoc = HotLocation::coffeemachine;
                } else {
                  nh.logerror("want cupdispenser but " + (int)currentHotLoc);
                }
                break;
            case 2:
                // Go to location just under the diaphragm
                // 2 step movement
                if (currentHotLoc == HotLocation::coffeemachine) {
                  // Current location must be coffeemachine
                  controller.rotate(DISTANCE_COFFEEMACHINE_XYSWITCH, 0, 0);
                  controller.rotate(0, DISTANCE_XYSWITCH_DIAPHRAGM, 0);
                  currentHotLoc = HotLocation::diaphragm;
                } else {
                  nh.logerror("wan diaphragm but " + (int)currentHotLoc);
                }
                break;
            case 3:
                // Present drink
                if (currentHotLoc == HotLocation::diaphragm) {
                  // Current location must be diaphgragm
                  controller.rotate(0, DISTANCE_DIAPHRAGM_PRESENT, 0);
                  currentHotLoc = HotLocation::present;
                } else {
                  nh.logerror("want presentation but " + (int)currentHotLoc);
                }
                break;
            default:
                nh.logerror("message.location not in range 0-3.");
        }
    }
    // Cold drink lane
    else if (message.lane == 2) {
        switch (message.location) {
            case 0:
              if (currentColdLoc == ColdLocation::present) {
                controller.rotate(0, 0, -(DISTANCE_DIAPHRAGM_PRESENT + DISTANCE_FULLYDOWN_DIAPHRAGM) + DISTANCE_RECEIVECAN_FULLYDOWN);
                currentColdLoc = ColdLocation::receiveCanHeight;
              } else {
                nh.logerror("want receiveCanHeight but " +(int) currentColdLoc);
              }
              break;
            case 1:
                // Go down so can will flip upright
                if (currentColdLoc == ColdLocation::receiveCanHeight) {
                  // Current location must be receiveCanHeight
                  controller.rotate(0, 0, DISTANCE_RECEIVECAN_FULLYDOWN);
                  currentColdLoc = ColdLocation::fullyDown;
                } else {
                  nh.logerror("want fullyDown but " +(int) currentColdLoc);
                }
                break;
            case 2:
                // Go to location just under the diaphragm
                if (currentColdLoc == ColdLocation::fullyDown) {
                  // Current location must be fullyDown
                  controller.rotate(0, 0, DISTANCE_FULLYDOWN_DIAPHRAGM);
                  currentColdLoc = ColdLocation::diaphragm;
                } else {
                  nh.logerror("want diaphragm but " +(int) currentColdLoc);
                }
                break;
            case 3:
                // Present drink
                if (currentColdLoc == ColdLocation::diaphragm) {
                  // Current location must be diaphragm
                  controller.rotate(0, 0, DISTANCE_DIAPHRAGM_PRESENT);
                  currentColdLoc = ColdLocation::present;
                } else {
                  nh.logerror("want present but " + (int)currentColdLoc);
                }
                break;
            default:
                nh.logerror("message.location not in range 0-3.");
        }
    } else {
        nh.logerror("message.lane not properly specified.");
    }
}

void zeroRequest(const std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    // TODO: Do more stuff

}

void interrupt_cold(){
   nh.logerror("Cold Interrupt");
}

void interrupt_hot_x(){
   nh.logerror("Hot X interrupt");
}

void interrupt_hot_y(){
   nh.logerror("Hot Y interrupt");
}

void setup() {
    // Init LED 'cause why not?
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(INTERRUPT_PIN_COLD, INPUT_PULLUP);
    pinMode(INTERRUPT_PIN_HOTX, INPUT_PULLUP);
    pinMode(INTERRUPT_PIN_HOTY, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_COLD), interrupt_cold, FALLING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_HOTX), interrupt_hot_x, FALLING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_HOTY), interrupt_hot_y, FALLING);
    digitalWrite(LED_BUILTIN, LOW);

    // Initiate USB serial connection used for ROS
    Serial.begin(57600);

    // Initialise all stepper motor instances
    stepper_hot_X.begin(MOTOR_HOT_X_RPM, MICROSTEPS);
    stepper_hot_Y.begin(MOTOR_HOT_Y_RPM, MICROSTEPS);
    stepper_cold.begin(MOTOR_COLD_RPM, MICROSTEPS);

    // Initialise ROS, its subscribers, publishers and services
    ROS_init();

    // Init locations to zero positions, ready to receive order
    currentHotLoc = HotLocation::cupdispenser;
    currentColdLoc = ColdLocation::receiveCanHeight;
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
