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

void log_error(char* desiredLocation, int requiredStart, int currentLocation) {
  char log_msg [100];
  sprintf(log_msg, "Want to go to %s, need to be at %d but I am at %d", desiredLocation, requiredStart, currentLocation);
  nh.logerror(log_msg);
}

void move_hotdrink(int location) {
  switch (location) {
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
            log_error("cupDispenser (hotx)", (int) HotXLocation::xyswitch, (int)currentHotXLoc);
            log_error("cupDispenser (hoty)", (int) HotYLocation::present, (int)currentHotYLoc);
          }
          break;
      case 1:
          // Go to coffee machine
          if (currentHotXLoc == HotXLocation::cupDispenser) {
            // Current location must be cupdispenser
            move_motor(MOTOR_HOT_X, (int)HotXLocation::coffeeMachine);
          } else {
            log_error("coffeeMachine", (int) HotXLocation::cupDispenser, (int)currentHotXLoc);
          }
          break;
      case 2:
          // Go to location just under the diaphragm
          // 2 step movement
          if (currentHotXLoc == HotXLocation::coffeeMachine && currentHotYLoc == HotYLocation::restLocation) {
            // Current location must be coffeemachine
            // Go to the switch position on x axis
            move_motor(MOTOR_HOT_X, (int)HotXLocation::xyswitch);
            // Then move up y axis to under diaphragm
            move_motor(MOTOR_HOT_Y, (int)HotYLocation::diaphragm);
          } else {
            log_error("diaphragm (hotx)", (int) HotXLocation::coffeeMachine, (int)currentHotXLoc);
            log_error("diaphragm (hoty)", (int) HotYLocation::restLocation, (int)currentHotYLoc);
          }
          break;
      case 3:
          // Present drink
          if (currentHotYLoc == HotYLocation::diaphragm) {
            // Current location must be diaphgragm
            move_motor(MOTOR_HOT_Y, (int)HotYLocation::present);
          } else {
            log_error("present", (int) HotYLocation::diaphragm, (int)currentHotYLoc);
          }
          break;
      default:
          nh.logerror("message.location not in range 0-3.");
  }
}

void move_colddrink(int location) {
  switch (location) {
      case 0:
        // Go to receive can height
        if (currentColdLoc == ColdLocation::present) {
          // Current location must be present
          move_motor(MOTOR_COLD, (int)ColdLocation::receiveCanHeight);
        } else {
          log_error("receiveCanHeight", (int) ColdLocation::present, (int)currentColdLoc);
        }
        break;
      case 1:
          // Go down so can will flip upright
          if (currentColdLoc == ColdLocation::receiveCanHeight) {
            // Current location must be receiveCanHeight
            move_motor(MOTOR_COLD, (int)ColdLocation::canLockin);
          } else {
            log_error("canLockin", (int) ColdLocation::receiveCanHeight, (int)currentColdLoc);
          }
          break;
      case 2:
          // Go to location just under the diaphragm
          if (currentColdLoc == ColdLocation::canLockin) {
            // Current location must be restLocation
            move_motor(MOTOR_COLD, (int)ColdLocation::diaphragm);
          } else {
            log_error("diaphragm", (int) ColdLocation::canLockin, (int)currentColdLoc);
          }
          break;
      case 3:
          // Present drink
          if (currentColdLoc == ColdLocation::diaphragm) {
            // Current location must be diaphgragm
            move_motor(MOTOR_COLD, (int)ColdLocation::present);
          } else {
            log_error("present", (int) ColdLocation::diaphragm, (int)currentColdLoc);
          }
          break;
      default:
          nh.logerror("message.location not in range 0-3.");
  }
}

void move_callback(const barrieduino::Move &message) {
    if (message.lane == 1) {
      // Hot drinks lane
      move_hotdrink(message.location);
    } else if (message.lane == 2) {
      // Cold drink lane
      move_colddrink(message.location);
    } else {
        nh.logerror("message.lane not properly specified.");
    }
}

void zeroRequest(const std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    // TODO: Move infinitely?
  controller.rotate(-100000, // Hot x -> go toward coffee machine
                    -100000, // Hot Y -> go fully down
                    -100000 // Cold ->  go fully down
  );
  nh.loginfo("Zeroed");
  move_motor(MOTOR_HOT_X, (int) HotXLocation::coffeeMachine);
  move_motor(MOTOR_HOT_Y, (int) HotYLocation::restLocation);
  move_motor(MOTOR_COLD, (int) ColdLocation::receiveCanHeight);
}

void setZero() {
  controller.rotate(-10000, // Hot x -> go toward coffee machine
                    -10000, // Hot Y -> go fully down
                    -10000 // Cold ->  go fully down
  );
  nh.loginfo("Zeroed");
  move_motor(MOTOR_HOT_X, (int) HotXLocation::coffeeMachine);
  move_motor(MOTOR_HOT_Y, (int) HotYLocation::restLocation);
  move_motor(MOTOR_COLD, (int) ColdLocation::receiveCanHeight);
}


long debouncing_time = 1000; //Debouncing Time in Milliseconds
volatile unsigned long last_micros_cold = 0;
volatile unsigned long last_micros_hotx = 0;
volatile unsigned long last_micros_hoty = 0;

void debounce_interrupt_cold() {
  if((long)(micros() - last_micros_cold) >= debouncing_time * 1000) {
    interrupt_cold();
    last_micros_cold = micros();
  }
}

void interrupt_cold(){
   nh.logerror("Cold Interrupt");
   char log_msg [10];
   sprintf(log_msg, "%d", (int)ColdLocation::canLockin);
   nh.loginfo(log_msg);
   stepper_cold.stop();
   currentColdLoc = ColdLocation::zeroLocation;
}

void debounce_interrupt_hot_x() {
  if((long)(micros() - last_micros_hotx) >= debouncing_time * 1000) {
    last_micros_hotx = micros();
    interrupt_hot_x();
  }
}

void interrupt_hot_x(){
   nh.logerror("Hot X interrupt");
   currentHotXLoc = HotXLocation::zeroLocation;
   stepper_hot_X.stop();
}

void debounce_interrupt_hot_y() {
  if((long)(micros() - last_micros_hoty) >= debouncing_time * 1000) {
    interrupt_hot_y();
    last_micros_hoty = micros();
  }
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
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_COLD), debounce_interrupt_cold, FALLING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_HOTX), debounce_interrupt_hot_x, FALLING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_HOTY), debounce_interrupt_hot_y, FALLING);
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

    //while(!nh.connected()) nh.spinOnce();
    //TODO remove inits
    //  setZero();
    // currentColdLoc = ColdLocation::receiveCanHeight;
    // currentHotYLoc = HotYLocation::restLocation;
    // currentHotXLoc = HotXLocation::cupDispenser;

    nh.loginfo("Arduino: Startup complete");
}

void loop() {
    // Do all necessary ROS synchronisations (at least once per 5 seconds)
    nh.spinOnce();

    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}
