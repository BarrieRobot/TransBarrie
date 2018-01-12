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
#define ENABLE_PIN_HOTX 10
#define ENABLE_PIN_HOTY 10
#define ENABLE_PIN_COLD 10

#define MOTOR_HOT_X 0
#define MOTOR_HOT_Y 1
#define MOTOR_COLD 2

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for hot drinks X axis motor
#define MOTOR_HOT_X_RPM 120
// Target RPM for hot drinks Y axis motor
#define MOTOR_HOT_Y_RPM 90
// Target RPM for cold drinks motor
#define MOTOR_COLD_RPM 90

#define MOTOR_ACCEL 250
#define MOTOR_DECEL 250

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

#define PULLEY_DIA 2.5 // Pulley diameter in cm

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
//BasicStepperDriver stepper_hot_X(MOTOR_STEPS, DIR_HOT_X, STEP_HOT_X);

/* Hot transport distances  */
DRV8825 stepper_hot_X(MOTOR_STEPS, DIR_HOT_X, STEP_HOT_X, ENABLE_PIN_HOTX);
DRV8825 stepper_hot_Y(MOTOR_STEPS, DIR_HOT_Y, STEP_HOT_Y, ENABLE_PIN_HOTY);
DRV8825 stepper_cold(MOTOR_STEPS, DIR_COLD, STEP_COLD, ENABLE_PIN_COLD);

MultiDriver controller(stepper_hot_X, stepper_hot_Y, stepper_cold);

// Robot Operating System
ros::NodeHandle nh;
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> zero_service("zeroRequest", &zeroRequest);
ros::Subscriber<barrieduino::Move> movement_sub("barrie_movement", &move_callback);

/* Locations in cm */
/* Hot X axis, towards user is positive movement */
enum class HotXLocation {
  zeroLocation = 0, // location near coffee machine,
  coffeeMachine = 20,
  cupDispenser = 40,
  xyswitch = 100,
};

/* Hot Y axis, upwards is positive movement */
enum class HotYLocation {
  zeroLocation = 0, // fully down
  restLocation = 5, // rest location above interrupt switch, may not be required
  diaphragm = 70,
  present = 100
};

/* Cold Y axis movement, upwards is positive */
enum class ColdLocation {
  zeroLocation = 0, // farmost down
  canLockin = 5, // lockin location above interrupt switch, may not be required
  receiveCanHeight = 20,
  diaphragm = 70,
  present = 100
};

/*/-- Code --/*/
HotYLocation currentHotYLoc; // Initialise when zeroed
HotXLocation currentHotXLoc;
ColdLocation currentColdLoc;

void ROS_init() {
    // Initialise ROS node and add subscribe to topics
    nh.initNode();
    nh.subscribe(movement_sub);
    nh.advertiseService(zero_service);
}

long get_degrees(long movement_cm) {
  return 360 * movement_cm / (PULLEY_DIA * PI);
}

void move_motor(int motor, int destination) {
  char log_msg [100];
  if (motor == MOTOR_HOT_X) {
    stepper_hot_X.enable();
    long degrees = get_degrees(destination - (int)currentHotXLoc);
    sprintf(log_msg, "Moving Hot X %d degrees", degrees);
    nh.loginfo(log_msg);
    controller.rotate(degrees, 0l, 0l);
    currentHotXLoc = static_cast<HotXLocation>(destination);
    stepper_hot_X.disable();
  } else if (motor == MOTOR_HOT_Y) {
    stepper_hot_Y.enable();
    long degrees = destination - (int)currentHotYLoc;
    sprintf(log_msg, "Moving Hot Y %d degrees", degrees);
    nh.loginfo(log_msg);
    controller.rotate(0l, degrees, 0l);
    currentHotYLoc = static_cast<HotYLocation>(destination);
    stepper_hot_X.disable();
  } else if (motor == MOTOR_COLD) {
    stepper_cold.enable();
    long degrees = destination - (int)currentColdLoc;
    sprintf(log_msg, "Moving Cold %d degrees", degrees);
    nh.loginfo(log_msg);
    controller.rotate(0l, 0l, degrees);
    currentColdLoc = static_cast<ColdLocation>(destination);
    stepper_hot_Y.disable();
  }
}

void move_callback(const barrieduino::Move &message) {
    // Hot drinks lane
    if (message.lane == 1) {
        switch (message.location) {
            case 0:
                // Go to cup dispenser
                // 2 step movement
                if (currentHotYLoc == HotYLocation::present && currentHotXLoc == HotXLocation::xyswitch) {
                  // Current location must be present and xyswitch
                  // first move down
                  move_motor(MOTOR_HOT_Y, (int)HotYLocation::restLocation);
                  // then to cupDispenser
                  move_motor(MOTOR_HOT_X, (int)HotXLocation::cupDispenser);
                } else {
                  char log_msg [100];
                  char log_msg2 [100];
                  sprintf(log_msg, "Want to go to cupDispenser, need to be at %d but I am at %d (hot x)", (int) HotXLocation::xyswitch, (int)currentHotXLoc);
                  sprintf(log_msg2, "Want to go to cupDispenser, need to be at %d but I am at %d (hot y)", (int) HotYLocation::present, (int)currentHotYLoc);
                  nh.logerror(log_msg);
                  nh.logerror(log_msg2);;
                }
                break;
            case 1:
                // Go to coffee machine
                if (currentHotXLoc == HotXLocation::cupDispenser) {
                  // Current location must be cupdispenser
                  move_motor(MOTOR_HOT_X, (int)HotXLocation::coffeeMachine);
                } else {
                  char log_msg [100];
                  sprintf(log_msg, "Want cupdispenser, need to be at %d but I am at %d", (int) HotXLocation::cupDispenser, (int)currentHotXLoc);
                  nh.logerror(log_msg);
                }
                break;
            case 2:
                // Go to location just under the diaphragm
                // 2 step movement
                if (currentHotXLoc == HotXLocation::coffeeMachine && currentHotYLoc == HotYLocation::restLocation) {
                  // Current location must be coffeemachine
                  move_motor(MOTOR_HOT_X, (int)HotXLocation::xyswitch);

                  move_motor(MOTOR_HOT_Y, (int)HotYLocation::diaphragm);
                } else {
                  char log_msg [100];
                  char log_msg2 [100];
                  sprintf(log_msg, "Want to go to diaphragm, need to be at %d but I am at %d (hot x)", (int) HotXLocation::coffeeMachine, (int)currentHotXLoc);
                  sprintf(log_msg2, "Want to go to diaphragm, need to be at %d but I am at %d (hot y)", (int) HotYLocation::restLocation, (int)currentHotYLoc);
                  nh.logerror(log_msg);
                  nh.logerror(log_msg2);
                }
                break;
            case 3:
                // Present drink
                if (currentHotYLoc == HotYLocation::diaphragm) {
                  // Current location must be diaphgragm
                  move_motor(MOTOR_HOT_Y, (int)HotYLocation::present);
                } else {
                  char log_msg [100];
                  sprintf(log_msg, "Want to go to present, need to be at %d but I am at %d", (int) HotYLocation::diaphragm, (int)currentHotYLoc);
                  nh.logerror(log_msg);
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
                // Current location must be present
                move_motor(MOTOR_COLD, (int)ColdLocation::receiveCanHeight);
              } else {
                char log_msg [100];
                sprintf(log_msg, "Want to go to receiveCanHeight, need to be at %d but I am at %d", (int) ColdLocation::present, (int)currentColdLoc);
                nh.logerror(log_msg);
              }
              break;
            case 1:
                // Go down so can will flip upright
                if (currentColdLoc == ColdLocation::receiveCanHeight) {
                  // Current location must be receiveCanHeight
                  move_motor(MOTOR_COLD, (int)ColdLocation::canLockin);
                } else {
                  char log_msg [100];
                  sprintf(log_msg, "Want to go to canLockin, need to be at %d but I am at %d", (int) ColdLocation::receiveCanHeight, (int)currentColdLoc);
                  nh.logerror(log_msg);
                }
                break;
            case 2:
                // Go to location just under the diaphragm
                if (currentColdLoc == ColdLocation::canLockin) {
                  // Current location must be restLocation
                  move_motor(MOTOR_COLD, (int)ColdLocation::diaphragm);
                } else {
                  char log_msg [100];
                  sprintf(log_msg, "Want to go to diaphragm, need to be at %d but I am at %d", (int) ColdLocation::canLockin, (int)currentColdLoc);
                  nh.logerror(log_msg);
                }
                break;
            case 3:
                // Present drink
                if (currentColdLoc == ColdLocation::diaphragm) {
                  // Current location must be diaphgragm
                  move_motor(MOTOR_COLD, (int)ColdLocation::present);
                } else {
                  char log_msg [100];
                  sprintf(log_msg, "Want to go to present, need to be at %d but I am at %d", (int) ColdLocation::diaphragm, (int)currentColdLoc);
                  nh.logerror(log_msg);
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
    // TODO: Move infinitely?
  controller.rotate(-1000, // Hot x -> go toward coffee machine
                    -1000, // Hot Y -> go fully down
                    -1000 // Cold ->  go fully down
  );

}

void interrupt_cold(){
   nh.logerror("Cold Interrupt");
   char log_msg [10];
   sprintf(log_msg, "%d", (int)ColdLocation::canLockin);
   nh.loginfo(log_msg);
   stepper_cold.stop();
   currentColdLoc = ColdLocation::zeroLocation;
}

void interrupt_hot_x(){
   nh.logerror("Hot X interrupt");
   stepper_hot_X.stop();
   currentHotXLoc = HotXLocation::zeroLocation;
}

void interrupt_hot_y(){
   nh.logerror("Hot Y interrupt");
   stepper_hot_Y.stop();
   currentHotYLoc = HotYLocation::zeroLocation;
   //move_motor(MOTOR_HOT_Y, HotYLocation::restLocation);
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

    stepper_hot_X.setSpeedProfile(stepper_hot_X.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
    stepper_hot_Y.setSpeedProfile(stepper_hot_Y.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
    stepper_cold.setSpeedProfile(stepper_cold.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

    // Initialise ROS, its subscribers, publishers and services
    ROS_init();

    // Init locations to zero positions, ready to receive order
    //while(!nh.connected()) nh.spinOnce();
    // controller.rotate(1000, 0, 0);

    //TODO remove inits
    currentColdLoc = ColdLocation::receiveCanHeight;
    currentHotYLoc = HotYLocation::restLocation;
    currentHotXLoc = HotXLocation::cupDispenser;

    nh.loginfo("Arduino: Startup complete");
}

void loop() {
    // Do all necessary ROS synchronisations (at least once per 5 seconds)
    nh.spinOnce();

    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}
