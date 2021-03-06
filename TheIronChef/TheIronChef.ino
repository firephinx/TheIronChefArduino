// ARDUINO MEGA PIN LAYOUT DEFINES
//#define 0
//#define 1
#define EndEffectorServo 2 // Interrupt Pin
#define ArmElbowServo 3 // Interrupt Pin
#define ArmTurntableServo 4 // PWM Pin
#define ElectromagnetRelay 5 // PWM Pin
//#define 5 // PWM Pin
//#define 6 // PWM Pin
//#define 7 // PWM Pin
//#define 8 // PWM Pin
//#define 9 // PWM Pin
//#define 10 // PWM Pin
//#define 11 // PWM Pin
//#define 12 // PWM Pin
//#define 13 // PWM Pin
//#define 14
//#define 15
//#define 16
//#define 17
//#define 18 // Interrupt Pin
//#define 19 // Interrupt Pin
//#define 20 // Interrupt Pin, SDA
//#define 21 // Interrupt Pin, SCL
#define XGantryStepper1Enable 22 
#define XGantryStepper2Enable 23
#define XGantryStepper1Direction 24
#define XGantryStepper2Direction 25
#define XGantryStepper1Pulse 26
#define XGantryStepper2Pulse 27
#define YGantryStepperEnable 28
#define ZGantryStepperEnable 29
#define YGantryStepperDirection 30
#define ZGantryStepperDirection 31
#define YGantryStepperPulse 32
#define ZGantryStepperPulse 33
//#define 34
//#define 35
//#define 36
//#define 37
//#define 38
//#define 39
//#define 40
//#define 41
//#define 42
//#define 43
//#define 44
//#define 45
//#define 46
//#define 47
#define XAxisLimitSwitch1 48
#define XAxisLimitSwitch2 49
#define YAxisLimitSwitch 50 // MISO
#define ZAxisLimitSwitch 51 // MOSI
//#define 52 // SCK
//#define 53 // SS

#define CONTINUOUS true

#include <EEPROM.h>
#include "EEPROMAnything.h"

struct gantry_positions_t
{
    long x_gantry_step_count;
    float x_gantry_position;
    long y_gantry_step_count;
    float y_gantry_position;
    long z_gantry_step_count;
    float z_gantry_position;
} gantry_positions;

// SERVO INCLUDES
#include <Servo.h>

// ROS INCLUDES
#include <ros.h> // ROS Serial
#include <ros/time.h> // ROS time for publishing messages
#include <std_msgs/Empty.h> // Empty msg for Homing, Resetting, and Staying in place
#include <std_msgs/Bool.h> // Bool msg for Electromagnet relay
#include <geometry_msgs/Point.h> // Point msg for moving the X, Y, Z Gantry, and Arm servos
#include <sensor_msgs/JointState.h> // JointState msg for publishing the robot's current joint values

// ROS Globals
ros::NodeHandle nh;

// Joint States Publisher Globals
sensor_msgs::JointState joint_states_msg;
char *joint_names[] = {"X_Gantry", "Y_Gantry", "Z_Gantry", "Arm_Turntable", "Arm_Elbow", "End_Effector"};
float joint_state_positions[6];
float joint_state_velocities[6];
float joint_state_efforts[6];

// Joint States Publisher
ros::Publisher joint_states_pub("/TheIronChef/joint_states", &joint_states_msg);

// X Gantry Globals
const int num_x_gantry_steps_from_limit_switch = 300;
const int x_gantry_step_interval = 1000;
const int x_gantry_step_time = 2000;
const int x_gantry_steps_per_revolution = 400;
const float x_gantry_distance_per_revolution = 0.005; // 5 mm pitch
const float x_gantry_length = 1.1176; // 1.1176m or 44"
const long max_x_gantry_steps = (long)(x_gantry_length / x_gantry_distance_per_revolution) * x_gantry_steps_per_revolution;
const float x_gantry_threshold = x_gantry_distance_per_revolution / x_gantry_steps_per_revolution;

// Y Gantry Globals
const int num_y_gantry_steps_from_limit_switch = 300;
const int y_gantry_step_interval = 1000;
const int y_gantry_step_time = 1000;
const int y_gantry_steps_per_revolution = 400;
const float y_gantry_distance_per_revolution = 0.005; // 5 mm pitch
const float y_gantry_length = 0.762; // 0.762m or 30"
const long max_y_gantry_steps = (long)(y_gantry_length / y_gantry_distance_per_revolution) * y_gantry_steps_per_revolution;
const float y_gantry_threshold = y_gantry_distance_per_revolution / y_gantry_steps_per_revolution;

// Z Gantry Globals

const int num_z_gantry_steps_from_limit_switch = 300;
const int z_gantry_step_interval = 1000;
const int z_gantry_step_time = 1000;
const int z_gantry_steps_per_revolution = 400;
const float z_gantry_distance_per_revolution = 0.005; // 5 mm pitch
const float z_gantry_length = 0.3302; // 0.3302m or 13"
const long max_z_gantry_steps = (long)(z_gantry_length / z_gantry_distance_per_revolution) * z_gantry_steps_per_revolution;
const float z_gantry_threshold = z_gantry_distance_per_revolution / z_gantry_steps_per_revolution;

// Arm Turntable Globals
const int default_arm_turntable_degree = 87;
int current_arm_turntable_degree = default_arm_turntable_degree;
Servo arm_turntable_servo;

// Arm Elbow Globals
const int default_arm_elbow_degree = 87;
int current_arm_elbow_degree = default_arm_elbow_degree;
Servo arm_elbow_servo;

// End Effector Globals
const int default_end_effector_degree = 70;
int current_end_effector_degree = default_end_effector_degree;
Servo end_effector_servo;

// ROS Callback Functions and Subscribers

// ELECTROMAGNET SWITCH

// electromagnet status Publisher
std_msgs::Bool electromagnet_status_msg;
ros::Publisher electromagnet_status_pub("/TheIronChef/electromagnet_status", &electromagnet_status_msg);

// Electromagnet Switch Callback
// Turns on the electromagnet if the msg contains true, 
// otherwise turns off the electromaget if the msg contains false
void electromagnetSwitchCallback(const std_msgs::Bool& electromagnet_switch_msg){
  // If the electromagnet_switch_msg contains true, turn on the electromagnet
  if(electromagnet_switch_msg.data)
  {
    digitalWrite(ElectromagnetRelay, LOW);
  }
  // Otherwise turn off the electromagnet
  else
  {
    digitalWrite(ElectromagnetRelay, HIGH);
  }
  electromagnet_status_msg.data = electromagnet_switch_msg.data;
  
  electromagnet_status_pub.publish(&electromagnet_status_msg);
  electromagnet_status_pub.publish(&electromagnet_status_msg);
}

// Electromagnet Switch Subscriber
ros::Subscriber<std_msgs::Bool> electromagnet_switch_sub("/TheIronChef/Electromagnet_Switch", &electromagnetSwitchCallback);

// STOP COMMAND
// Stop Command Globals
bool stop_flag = false;

// Stop Command Callback
// Immediately halts everything by setting all the stepper motor enable pins to HIGH and sets the stop_flag to true.
// Remember to publish a false msg to restart the robot.
void stopCallback(const std_msgs::Bool& stop_msg){

  // If the stop_msg contains true, stop all of the motors
  if(stop_msg.data)
  {
    // set stop_flag to true
    stop_flag = true;

    // disable all stepper motors by setting all the enable pins to HIGH
    digitalWrite(XGantryStepper1Enable, HIGH);
    digitalWrite(XGantryStepper2Enable, HIGH);
    digitalWrite(YGantryStepperEnable, HIGH);
    digitalWrite(ZGantryStepperEnable, HIGH);
    
    // electromagnet by setting their switches to LOW
    digitalWrite(ElectromagnetRelay, LOW);
  }
  // Otherwise if the stop_msg contains false, set the stop_flag back to false
  // and enable the stepper motors.
  else
  {
    stop_flag = false;
  }
}

// Stop Command Subscriber
ros::Subscriber<std_msgs::Bool> stop_sub("/TheIronChef/Stop", &stopCallback);

// HOME COMMAND
// Home Command Globals
bool home_x_gantry_flag = false;
bool home_y_gantry_flag = false;
bool home_z_gantry_flag = false;
bool home_arm_turntable_flag = false;
bool home_arm_elbow_flag = false;
bool home_end_effector_flag = false;

// done homing Publisher
std_msgs::Bool done_homing_msg;
ros::Publisher done_homing_pub("/TheIronChef/done_homing", &done_homing_msg);

// Home Command Callback
// Returns the robot to the home position by setting the homing flags to true
void homeCallback(const std_msgs::Empty& home_msg){
  home_x_gantry_flag = true;
  home_y_gantry_flag = true;
  home_z_gantry_flag = true;
  home_arm_turntable_flag = true;
  home_arm_elbow_flag = true;
  home_end_effector_flag = true;
}

// Home Command Subscriber
ros::Subscriber<std_msgs::Empty> home_sub("/TheIronChef/Home", &homeCallback);

// Home Arm Command Callback
// Returns the robot arm to the home position by setting the home arm flags to true
void homeArmCallback(const std_msgs::Empty& home_arm_msg){
  home_arm_turntable_flag = true;
  home_arm_elbow_flag = true;
  home_end_effector_flag = true;
}

// Home Arm Command Subscriber
ros::Subscriber<std_msgs::Empty> home_arm_sub("/TheIronChef/HomeArm", &homeArmCallback);

// Home Gantry Command Callback
// Returns the gantry to the home position by setting the home gantry flags to true
void homeGantryCallback(const std_msgs::Empty& home_arm_msg){
  home_x_gantry_flag = true;
  home_y_gantry_flag = true;
  home_z_gantry_flag = true;
}

// Home Gantry Command Subscriber
ros::Subscriber<std_msgs::Empty> home_gantry_sub("/TheIronChef/HomeGantry", &homeGantryCallback);

// Home X Gantry Command Callback
// Homes the X Gantry by setting the x_gantry_flag to true
void homeXGantryCallback(const std_msgs::Empty& home_x_gantry_msg){
  home_x_gantry_flag = true;
}

// Home X Gantry Command Subscriber
ros::Subscriber<std_msgs::Empty> home_x_gantry_sub("/TheIronChef/HomeXGantry", &homeXGantryCallback);

// Home Y Gantry Command Callback
// Homes the Y Gantry by setting the y_gantry_flag to true
void homeYGantryCallback(const std_msgs::Empty& home_y_gantry_msg){
  home_y_gantry_flag = true;
}

// Home Y Gantry Command Subscriber
ros::Subscriber<std_msgs::Empty> home_y_gantry_sub("/TheIronChef/HomeYGantry", &homeYGantryCallback);

// Home Z Gantry Command Callback
// Homes the Z Gantry by setting the z_gantry_flag to true
void homeZGantryCallback(const std_msgs::Empty& home_z_gantry_msg){
  home_z_gantry_flag = true;
}

// Home Z Gantry Command Subscriber
ros::Subscriber<std_msgs::Empty> home_z_gantry_sub("/TheIronChef/HomeZGantry", &homeZGantryCallback);

// MOVE GANTRY
// Move Gantry Globals
bool move_x_gantry_flag = false;
bool move_y_gantry_flag = false;
bool move_z_gantry_flag = false;
float desired_x_gantry_position;
float desired_y_gantry_position;
float desired_z_gantry_position;

// done moving gantry Publisher
std_msgs::Bool done_moving_gantry_msg;
ros::Publisher done_moving_gantry_pub("/TheIronChef/done_moving_gantry", &done_moving_gantry_msg);

// Move Gantry Callback
void moveGantryCallback(const geometry_msgs::Point& move_gantry_msg)
{
  desired_x_gantry_position = move_gantry_msg.x;
  desired_y_gantry_position = move_gantry_msg.y;
  desired_z_gantry_position = move_gantry_msg.z;
  
  if(abs(desired_x_gantry_position - gantry_positions.x_gantry_position) > x_gantry_threshold)
  {
    move_x_gantry_flag = true;
  } 
  if(abs(desired_y_gantry_position - gantry_positions.y_gantry_position) > y_gantry_threshold)
  {
    move_y_gantry_flag = true;
  } 
  if(abs(desired_z_gantry_position - gantry_positions.z_gantry_position) > z_gantry_threshold)
  {
    move_z_gantry_flag = true;
  }
  if(!move_x_gantry_flag && !move_y_gantry_flag && !move_z_gantry_flag)
  {
    done_moving_gantry_msg.data = true;
    done_moving_gantry_pub.publish(&done_moving_gantry_msg);
  }
}

// Move Gantry Subscriber
ros::Subscriber<geometry_msgs::Point> move_gantry_sub("/TheIronChef/move_gantry", &moveGantryCallback);

// MOVE ARM
// Move Arm Globals
bool turn_arm_turntable_flag = false;
bool move_arm_elbow_flag = false;
bool move_end_effector_flag = false;
int desired_arm_turntable_degree;
int desired_arm_elbow_degree;
int desired_end_effector_degree;

// done moving arm Publisher
std_msgs::Bool done_moving_arm_msg;
ros::Publisher done_moving_arm_pub("/TheIronChef/done_moving_arm", &done_moving_arm_msg);

// Move Arm Callback
void moveArmCallback(const geometry_msgs::Point& move_arm_msg)
{
  desired_arm_turntable_degree = (int)move_arm_msg.x;
  desired_arm_elbow_degree = (int)move_arm_msg.y;
  desired_end_effector_degree = (int)move_arm_msg.z;
  
  if(desired_arm_turntable_degree != current_arm_turntable_degree)
  {
    turn_arm_turntable_flag = true;
  }
  if(desired_arm_elbow_degree != current_arm_elbow_degree)
  {
    move_arm_elbow_flag = true;
  }
  if(desired_end_effector_degree != current_end_effector_degree)
  {
    move_end_effector_flag = true;
  }
  if(!turn_arm_turntable_flag && !move_arm_elbow_flag && !move_end_effector_flag)
  {
    done_moving_arm_msg.data = true;
    done_moving_arm_pub.publish(&done_moving_arm_msg);
  }
}

// Move Arm Subscriber
ros::Subscriber<geometry_msgs::Point> move_arm_sub("/TheIronChef/move_arm", &moveArmCallback);

// RESET COMMAND
// Reset Command Callback
// Resets the robot's desired positions to the default positions.
void resetCallback(const std_msgs::Empty& reset_msg){
  desired_x_gantry_position = 0.0;
  desired_y_gantry_position = 0.0;
  desired_z_gantry_position = 0.0;
  desired_arm_turntable_degree = default_arm_turntable_degree;
  desired_arm_elbow_degree = default_arm_elbow_degree;
  desired_end_effector_degree = default_end_effector_degree;
}

// Reset Command Subscriber
ros::Subscriber<std_msgs::Empty> reset_sub("/TheIronChef/Reset", &resetCallback);

// STAY COMMAND
// Stay Command Callback
// Sets the robot's desired positions to the current positions.
void stayCallback(const std_msgs::Empty& stay_msg){
  desired_x_gantry_position = gantry_positions.x_gantry_position;
  desired_y_gantry_position = gantry_positions.y_gantry_position;
  desired_z_gantry_position = gantry_positions.z_gantry_position;
  desired_arm_turntable_degree = current_arm_turntable_degree;
  desired_arm_elbow_degree = current_arm_elbow_degree;
  desired_end_effector_degree = current_end_effector_degree;
}

// Stay Command Subscriber
ros::Subscriber<std_msgs::Empty> stay_sub("/TheIronChef/Stay", &stayCallback);

// SET GANTRY COMMAND
// Set Gantry Callback
// Sets the robot's current gantry position to the ones sent in the message.
void setGantryCallback(const geometry_msgs::Point& set_gantry_msg){
  gantry_positions.x_gantry_position = set_gantry_msg.x;
  gantry_positions.y_gantry_position = set_gantry_msg.y;
  gantry_positions.z_gantry_position = set_gantry_msg.z;
  gantry_positions.x_gantry_step_count = (long)((gantry_positions.x_gantry_position / x_gantry_distance_per_revolution) * x_gantry_steps_per_revolution);
  gantry_positions.y_gantry_step_count = (long)((gantry_positions.y_gantry_position / y_gantry_distance_per_revolution) * y_gantry_steps_per_revolution);
  gantry_positions.z_gantry_step_count = (long)((gantry_positions.z_gantry_position / z_gantry_distance_per_revolution) * z_gantry_steps_per_revolution);
  EEPROM_writeAnything(0, gantry_positions);
}

// Stay Command Subscriber
ros::Subscriber<geometry_msgs::Point> set_gantry_sub("/TheIronChef/SetGantry", &setGantryCallback);


// SETUP CODE
void setup()
{ 
  // Read from EEPROM
  EEPROM_readAnything(0, gantry_positions);
  if(isnan(gantry_positions.x_gantry_step_count))
  {
    gantry_positions.x_gantry_step_count = 0;
    gantry_positions.x_gantry_position = 0.0;
  }
  if(isnan(gantry_positions.y_gantry_step_count))
  {
    gantry_positions.y_gantry_step_count = 0;
    gantry_positions.y_gantry_position = 0.0;
  }
  if(isnan(gantry_positions.z_gantry_step_count))
  {
    gantry_positions.z_gantry_step_count = 0;
    gantry_positions.z_gantry_position = 0.0;
  }

  // Set all the stepper motor control pins to outputs
  pinMode(XGantryStepper1Enable, OUTPUT);
  pinMode(XGantryStepper1Direction, OUTPUT);
  pinMode(XGantryStepper1Pulse, OUTPUT);
  pinMode(XGantryStepper2Enable, OUTPUT);
  pinMode(XGantryStepper2Direction, OUTPUT);
  pinMode(XGantryStepper2Pulse, OUTPUT);
  pinMode(YGantryStepperEnable, OUTPUT);
  pinMode(YGantryStepperDirection, OUTPUT);
  pinMode(YGantryStepperPulse, OUTPUT);
  pinMode(ZGantryStepperEnable, OUTPUT);
  pinMode(ZGantryStepperDirection, OUTPUT);
  pinMode(ZGantryStepperPulse, OUTPUT);

  // Attach all the servos
  arm_turntable_servo.attach(ArmTurntableServo);
  arm_elbow_servo.attach(ArmElbowServo);
  end_effector_servo.attach(EndEffectorServo);

  // Set the electromagnet relay to outputs
  pinMode(ElectromagnetRelay, OUTPUT);

  // Set limit switches to input pullup mode
  pinMode(XAxisLimitSwitch1, INPUT_PULLUP);
  pinMode(XAxisLimitSwitch2, INPUT_PULLUP);
  pinMode(YAxisLimitSwitch, INPUT_PULLUP); 
  pinMode(ZAxisLimitSwitch, INPUT_PULLUP); 

  // Disable stepper motors by setting their enable lines to high
  digitalWrite(XGantryStepper1Enable, HIGH);
  digitalWrite(XGantryStepper2Enable, HIGH);
  digitalWrite(YGantryStepperEnable, HIGH);
  digitalWrite(ZGantryStepperEnable, HIGH);

  // Turn off the electromagnet initially by setting the pin to high
  digitalWrite(ElectromagnetRelay, HIGH);

  // Set the Servos to their default positions
  //arm_turntable_servo.write(default_arm_turntable_degree);
  //arm_elbow_servo.write(default_arm_elbow_degree);
  //end_effector_servo.write(default_end_effector_degree);

  // ROS Serial Initialization Code

  // Initialize Node
  nh.initNode();
  
  // Advertise topics
  nh.advertise(joint_states_pub);
  nh.advertise(electromagnet_status_pub);
  nh.advertise(done_homing_pub);
  nh.advertise(done_moving_gantry_pub);
  nh.advertise(done_moving_arm_pub);
  

  // Subscribe to topics
  nh.subscribe(electromagnet_switch_sub);
  nh.subscribe(stop_sub);
  nh.subscribe(home_sub);
  nh.subscribe(home_arm_sub);
  nh.subscribe(home_gantry_sub);
  nh.subscribe(home_x_gantry_sub);
  nh.subscribe(home_y_gantry_sub);
  nh.subscribe(home_z_gantry_sub);
  nh.subscribe(reset_sub);
  nh.subscribe(stay_sub);
  nh.subscribe(set_gantry_sub);
  nh.subscribe(move_gantry_sub);
  nh.subscribe(move_arm_sub);

  // Joint States Msg Setup Code
  joint_states_msg.name_length = 6;
  joint_states_msg.velocity_length = 6;
  joint_states_msg.position_length = 6;
  joint_states_msg.effort_length = 6;
  joint_states_msg.name = joint_names;
}

bool determineHoming()
{
  return (home_x_gantry_flag || home_y_gantry_flag || home_z_gantry_flag || 
          home_arm_turntable_flag || home_arm_elbow_flag || home_end_effector_flag);
}

void checkIfDoneHoming()
{
  bool homing_flag = determineHoming();
  if(homing_flag == false)
  {
    done_homing_msg.data = true;
    done_homing_pub.publish(&done_homing_msg);
    done_homing_pub.publish(&done_homing_msg);
  }
}

void home()
{
  if(home_z_gantry_flag)
  {
    homeZGantry();
    checkIfDoneHoming();
  }
  else if(home_y_gantry_flag)
  {
    homeYGantry();
    checkIfDoneHoming();
  }
  else if(home_x_gantry_flag)
  {
    homeXGantry();
    checkIfDoneHoming();
  }
  else if(home_arm_turntable_flag)
  {
    homeArmTurntable();
    checkIfDoneHoming();
  }
  else if(home_arm_elbow_flag)
  {
    homeArmElbow();
    checkIfDoneHoming();
  }
  else if(home_end_effector_flag)
  {
    homeEndEffector();
    checkIfDoneHoming();
  }
}

void homeXGantry()
{
  int num_steps = 0;

  // Enable X Gantry Steppers
  digitalWrite(XGantryStepper1Enable, LOW);
  digitalWrite(XGantryStepper2Enable, LOW);

  // Move Backwards
  digitalWrite(XGantryStepper1Direction, HIGH);
  digitalWrite(XGantryStepper2Direction, HIGH);

  while(num_steps < x_gantry_step_interval && digitalRead(XAxisLimitSwitch1) != 0 && digitalRead(XAxisLimitSwitch2) != 0)
  {
    
    digitalWrite(XGantryStepper1Pulse, HIGH);
    digitalWrite(XGantryStepper2Pulse, HIGH);
    digitalWrite(XGantryStepper1Pulse, LOW);
    digitalWrite(XGantryStepper2Pulse, LOW);
    gantry_positions.x_gantry_step_count--;

    delayMicroseconds(x_gantry_step_time);
  }
  
  // Check to see if the X Axis Limit Switches were hit
  if(digitalRead(XAxisLimitSwitch1) == 0 && digitalRead(XAxisLimitSwitch2) == 0)
  {         
    // Conduct the X Gantry Calibration Sequence
    XGantryCalibrationSequence();

    // Set the gantry_positions.x_gantry_step_count to 0
    gantry_positions.x_gantry_step_count = 0;

    // Set the home_x_gantry_flag to false because the x_gantry was successfully homed
    home_x_gantry_flag = false;
  }
  else if(digitalRead(XAxisLimitSwitch1) == 0)
  {
    while(num_steps < x_gantry_step_interval && digitalRead(XAxisLimitSwitch2) != 0)
    {
      digitalWrite(XGantryStepper2Pulse, HIGH);
      digitalWrite(XGantryStepper2Pulse, LOW);

      delayMicroseconds(x_gantry_step_time);
    }
  }
  else if(digitalRead(XAxisLimitSwitch2) == 0)
  {
    while(num_steps < x_gantry_step_interval && digitalRead(XAxisLimitSwitch1) != 0)
    {
      digitalWrite(XGantryStepper1Pulse, HIGH);
      digitalWrite(XGantryStepper1Pulse, LOW);

      delayMicroseconds(x_gantry_step_time);
    }
  }

  // Update gantry_positions.x_gantry_position with the current gantry_positions.x_gantry_step_count
  gantry_positions.x_gantry_position = ((float)(gantry_positions.x_gantry_step_count) / x_gantry_steps_per_revolution) * x_gantry_distance_per_revolution;

  EEPROM_writeAnything(0, gantry_positions);

  // Disable X Gantry Steppers
  digitalWrite(XGantryStepper1Enable, HIGH);
  digitalWrite(XGantryStepper2Enable, HIGH);
}

// Conducts the X Gantry Calibration Sequence after the limit switch has been hit, which involves 
// moving the X Gantry out until the limit switch is no longer clicked.
void XGantryCalibrationSequence()
{
  digitalWrite(XGantryStepper1Direction, LOW);
  digitalWrite(XGantryStepper2Direction, LOW);

  while (digitalRead(XAxisLimitSwitch1) != 1 && digitalRead(XAxisLimitSwitch2) != 1)
  {
   digitalWrite(XGantryStepper1Pulse, HIGH);
   digitalWrite(XGantryStepper2Pulse, HIGH);
   digitalWrite(XGantryStepper1Pulse, LOW);
   digitalWrite(XGantryStepper2Pulse, LOW);
   
   delayMicroseconds(x_gantry_step_time);
  }
  for (int i = 1; i < num_x_gantry_steps_from_limit_switch; i++)
  {
   digitalWrite(XGantryStepper1Pulse, HIGH);
   digitalWrite(XGantryStepper2Pulse, HIGH);
   digitalWrite(XGantryStepper1Pulse, LOW);
   digitalWrite(XGantryStepper2Pulse, LOW);

   delayMicroseconds(x_gantry_step_time);
  } 
}

void homeYGantry()
{
  int num_steps = 0;

  // Enable Y Gantry Stepper
  digitalWrite(YGantryStepperEnable, LOW);

  // Move Backwards
  digitalWrite(YGantryStepperDirection, HIGH);

  while(num_steps < y_gantry_step_interval && digitalRead(YAxisLimitSwitch) != 0)
  {
    digitalWrite(YGantryStepperPulse, HIGH);
    digitalWrite(YGantryStepperPulse, LOW);
    gantry_positions.y_gantry_step_count--;

    delayMicroseconds(y_gantry_step_time);
  }
  
  // Check to see if the Y Axis Limit Switch was hit
  if(digitalRead(YAxisLimitSwitch) == 0)
  {         
    // Conduct the Y Gantry Calibration Sequence
    YGantryCalibrationSequence();

    // Set the gantry_positions.y_gantry_step_count to 0
    gantry_positions.y_gantry_step_count = 0;

    // Set the home_y_gantry_flag to false because the y_gantry was successfully homed
    home_y_gantry_flag = false;
  }

  // Update gantry_positions.x_gantry_position with the current gantry_positions.x_gantry_step_count
  gantry_positions.y_gantry_position = ((float)(gantry_positions.y_gantry_step_count) / y_gantry_steps_per_revolution) * y_gantry_distance_per_revolution;

  EEPROM_writeAnything(0, gantry_positions);

  // Disable Y Gantry Stepper
  digitalWrite(YGantryStepperEnable, HIGH);
}

// Conducts the Y Gantry Calibration Sequence after the limit switch has been hit, which involves 
// moving the Y Gantry out until the limit switch is no longer clicked.
void YGantryCalibrationSequence()
{
  digitalWrite(YGantryStepperDirection, LOW);
  while (digitalRead(YAxisLimitSwitch) != 1)
  {
   digitalWrite(YGantryStepperPulse, HIGH);
   digitalWrite(YGantryStepperPulse, LOW);
   
   delayMicroseconds(y_gantry_step_time);
  }
  for (int i = 1; i < num_y_gantry_steps_from_limit_switch; i++)
  {
   digitalWrite(YGantryStepperPulse, HIGH);
   digitalWrite(YGantryStepperPulse, LOW);

   delayMicroseconds(y_gantry_step_time);
  }
}

void homeZGantry()
{
  int num_steps = 0;

  // Enable Z Gantry Stepper
  digitalWrite(ZGantryStepperEnable, LOW);

  // Move Backward
  digitalWrite(ZGantryStepperDirection, HIGH);

  while(num_steps < z_gantry_step_interval && digitalRead(ZAxisLimitSwitch) != 0)
  {
    digitalWrite(ZGantryStepperPulse, HIGH);
    digitalWrite(ZGantryStepperPulse, LOW);
    gantry_positions.z_gantry_step_count--;

    delayMicroseconds(z_gantry_step_time);
  }

  // Check to see if the Z Axis Limit Switch was hit
  if(digitalRead(ZAxisLimitSwitch) == 0)
  {         
    // Conduct the Z Gantry Calibration Sequence
    ZGantryCalibrationSequence();
    
    // Set the gantry_positions.z_gantry_step_count to 0
    gantry_positions.z_gantry_step_count = 0;

    // Set the home_z_gantry_flag to false because the z_gantry was successfully homed
    home_z_gantry_flag = false;
  }

  // Update gantry_positions.z_gantry_position with the current gantry_positions.z_gantry_step_count
  gantry_positions.z_gantry_position = ((float)(gantry_positions.z_gantry_step_count) / z_gantry_steps_per_revolution) * z_gantry_distance_per_revolution;

  EEPROM_writeAnything(0, gantry_positions);

  // Disable Z Gantry Stepper
  digitalWrite(ZGantryStepperEnable, HIGH);
}

// Conducts the Z Gantry Calibration Sequence after the limit switch has been hit, which involves 
// moving the Z Gantry out until the limit switch is no longer clicked.
void ZGantryCalibrationSequence()
{
  digitalWrite(ZGantryStepperDirection, LOW);
  while (digitalRead(ZAxisLimitSwitch) != 1)
  {
   digitalWrite(ZGantryStepperPulse, HIGH);
   digitalWrite(ZGantryStepperPulse, LOW);
   
   delayMicroseconds(z_gantry_step_time);
  }
  for (int i = 1; i < num_z_gantry_steps_from_limit_switch; i++)
  {
   digitalWrite(ZGantryStepperPulse, HIGH);
   digitalWrite(ZGantryStepperPulse, LOW);

   delayMicroseconds(z_gantry_step_time);
  }
}

void homeArmTurntable()
{
  arm_turntable_servo.write(default_arm_turntable_degree);
  current_arm_turntable_degree = default_arm_turntable_degree;
  home_arm_turntable_flag = false;
}

void homeArmElbow()
{
  arm_elbow_servo.write(default_arm_elbow_degree);
  current_arm_elbow_degree = default_arm_elbow_degree;
  home_arm_elbow_flag = false;
}

void homeEndEffector()
{
  end_effector_servo.write(default_end_effector_degree);
  current_end_effector_degree = default_end_effector_degree;
  home_end_effector_flag = false;
}

// Publish the current joint state positions of the turntable, x-gantry, and z-gantry to ROS
void publishJointStates()
{
  joint_states_msg.header.stamp = nh.now();
  joint_state_positions[0] = gantry_positions.x_gantry_position;
  joint_state_positions[1] = gantry_positions.y_gantry_position;
  joint_state_positions[2] = gantry_positions.z_gantry_position;
  joint_state_positions[3] = (float)current_arm_turntable_degree;
  joint_state_positions[4] = (float)current_arm_elbow_degree;
  joint_state_positions[5] = (float)current_end_effector_degree;
  joint_state_velocities[0] = (float) digitalRead(XAxisLimitSwitch1);
  joint_state_velocities[1] = (float) digitalRead(XAxisLimitSwitch2);
  joint_state_velocities[2] = (float) digitalRead(YAxisLimitSwitch);
  joint_state_velocities[3] = (float) digitalRead(ZAxisLimitSwitch);
  joint_state_efforts[0] = (float) gantry_positions.x_gantry_step_count;
  joint_state_efforts[1] = (float) gantry_positions.y_gantry_step_count;
  joint_state_efforts[2] = (float) gantry_positions.z_gantry_step_count;
  joint_states_msg.position = joint_state_positions;
  joint_states_msg.velocity = joint_state_velocities;
  joint_states_pub.publish(&joint_states_msg);
}

void checkIfDoneMovingGantry()
{
  bool moving_gantry_flag = (move_x_gantry_flag || move_y_gantry_flag || move_z_gantry_flag);
  
  if(moving_gantry_flag == false)
  {
    done_moving_gantry_msg.data = true;
    done_moving_gantry_pub.publish(&done_moving_gantry_msg);
    done_moving_gantry_pub.publish(&done_moving_gantry_msg);
  }
}

void checkIfDoneMovingArm()
{
  bool moving_arm_flag = (turn_arm_turntable_flag || move_arm_elbow_flag || move_end_effector_flag);
  
  if(moving_arm_flag == false)
  {
    done_moving_arm_msg.data = true;
    done_moving_arm_pub.publish(&done_moving_arm_msg);
    done_moving_arm_pub.publish(&done_moving_arm_msg);
  }
}

// LOOP CODE
void loop()
{
  // If the robot is not currently in the stop mode
  if(!stop_flag)
  {
    if(determineHoming())
    {
      home();
    }
    else
    {
      if(move_x_gantry_flag || move_y_gantry_flag || move_z_gantry_flag)
      {
        if(move_x_gantry_flag)
        {
          moveXGantry();
          checkIfDoneMovingGantry();
        } 
        else if(move_y_gantry_flag)
        {
          moveYGantry();
          checkIfDoneMovingGantry();
        }
        else if(move_z_gantry_flag)
        {
          moveZGantry();
          checkIfDoneMovingGantry();
        }
      }
      else if(turn_arm_turntable_flag)
      {
        turnArmTurntable();
        checkIfDoneMovingArm();
      }
      else if(move_arm_elbow_flag)
      {
        moveArmElbow();
        checkIfDoneMovingArm();
      }
      else if(move_end_effector_flag)
      {
        moveEndEffector();
        checkIfDoneMovingArm();
      }
    }
  }
  
  publishJointStates();

  nh.spinOnce();
  delay(3);
}

#ifdef CONTINUOUS
void moveXGantry()
{
  // Enable X Gantry Steppers
  digitalWrite(XGantryStepper1Enable, LOW);
  digitalWrite(XGantryStepper2Enable, LOW);
  
  long num_x_gantry_steps = (long)(((desired_x_gantry_position - gantry_positions.x_gantry_position) / x_gantry_distance_per_revolution) * x_gantry_steps_per_revolution);

  if(num_x_gantry_steps > 0)
  {
    // Move X Gantry Forward
    digitalWrite(XGantryStepper1Direction, LOW);
    digitalWrite(XGantryStepper2Direction, LOW);

    for (long i = 0; i < num_x_gantry_steps; i++)
    {   
      // Check to see if the X Axis Limit Switches were hit
      if(digitalRead(XAxisLimitSwitch1) == 0 || digitalRead(XAxisLimitSwitch2) == 0)
      { 
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }
            
      if(gantry_positions.x_gantry_step_count >= max_x_gantry_steps)
      {
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }
      digitalWrite(XGantryStepper1Pulse, HIGH);
      digitalWrite(XGantryStepper2Pulse, HIGH);
      digitalWrite(XGantryStepper1Pulse, LOW);
      digitalWrite(XGantryStepper2Pulse, LOW);
      gantry_positions.x_gantry_step_count++;

      delayMicroseconds(x_gantry_step_time);
    }
  }
  else
  {
    // Move X Gantry Back
    digitalWrite(XGantryStepper1Direction, HIGH);
    digitalWrite(XGantryStepper2Direction, HIGH);

    for (long i = 0; i < -num_x_gantry_steps; i++)
    { 
      // Check to see if the X Axis Limit Switches were hit
      if(digitalRead(XAxisLimitSwitch1) == 0 || digitalRead(XAxisLimitSwitch2) == 0)
      { 
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }        
      if(gantry_positions.x_gantry_step_count == 0)
      {
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }
      digitalWrite(XGantryStepper1Pulse, HIGH);
      digitalWrite(XGantryStepper2Pulse, HIGH);
      digitalWrite(XGantryStepper1Pulse, LOW);
      digitalWrite(XGantryStepper2Pulse, LOW);
      gantry_positions.x_gantry_step_count--;

      delayMicroseconds(x_gantry_step_time);
    }
  }

  gantry_positions.x_gantry_position = ((float)(gantry_positions.x_gantry_step_count) / x_gantry_steps_per_revolution) * x_gantry_distance_per_revolution;

  EEPROM_writeAnything(0, gantry_positions);
  
  move_x_gantry_flag = false;
  // Disable X Gantry Steppers
  digitalWrite(XGantryStepper1Enable, HIGH);
  digitalWrite(XGantryStepper2Enable, HIGH);
}
#else
void moveXGantry()
{
  // Enable X Gantry Steppers
  digitalWrite(XGantryStepper1Enable, LOW);
  digitalWrite(XGantryStepper2Enable, LOW);
  
  long num_x_gantry_steps = (long)(((desired_x_gantry_position - gantry_positions.x_gantry_position) / x_gantry_distance_per_revolution) * x_gantry_steps_per_revolution);

  if(num_x_gantry_steps > 0)
  {
    // Move X Gantry Forward
    digitalWrite(XGantryStepper1Direction, LOW);
    digitalWrite(XGantryStepper2Direction, LOW);

    for (long i = 0; i < min(x_gantry_step_interval, num_x_gantry_steps); i++)
    {   
      // Check to see if the X Axis Limit Switches were hit
      if(digitalRead(XAxisLimitSwitch1) == 0 || digitalRead(XAxisLimitSwitch2) == 0)
      { 
/*        if(digitalRead(XAxisLimitSwitch1) == 0 && digitalRead(XAxisLimitSwitch2) == 0)
        {
        }
        else if(digitalRead(XAxisLimitSwitch1) == 0)
        {
          while(digitalRead(XAxisLimitSwitch2) != 0)
          {
            digitalWrite(XGantryStepper2Pulse, HIGH);
            digitalWrite(XGantryStepper2Pulse, LOW);
      
            delayMicroseconds(x_gantry_step_time);
          }
        }
        else if(digitalRead(XAxisLimitSwitch2) == 0)
        {
          while(digitalRead(XAxisLimitSwitch1) != 0)
          {
            digitalWrite(XGantryStepper1Pulse, HIGH);
            digitalWrite(XGantryStepper1Pulse, LOW);
      
            delayMicroseconds(x_gantry_step_time);
          }
        }
        // Conduct the X Gantry Calibration Sequence
        XGantryCalibrationSequence();
      
        // Set the gantry_positions.x_gantry_step_count to 0
        gantry_positions.x_gantry_step_count = 0;*/

        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }
            
      if(gantry_positions.x_gantry_step_count >= max_x_gantry_steps)
      {
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }
      digitalWrite(XGantryStepper1Pulse, HIGH);
      digitalWrite(XGantryStepper2Pulse, HIGH);
      digitalWrite(XGantryStepper1Pulse, LOW);
      digitalWrite(XGantryStepper2Pulse, LOW);
      gantry_positions.x_gantry_step_count++;

      delayMicroseconds(x_gantry_step_time);
    }
  }
  else
  {
    // Move X Gantry Back
    digitalWrite(XGantryStepper1Direction, HIGH);
    digitalWrite(XGantryStepper2Direction, HIGH);

    for (long i = 0; i < min(x_gantry_step_interval, -num_x_gantry_steps); i++)
    { 
      // Check to see if the X Axis Limit Switches were hit
      if(digitalRead(XAxisLimitSwitch1) == 0 || digitalRead(XAxisLimitSwitch2) == 0)
      { 
/*        if(digitalRead(XAxisLimitSwitch1) == 0 && digitalRead(XAxisLimitSwitch2) == 0)
        {
        }
        else if(digitalRead(XAxisLimitSwitch1) == 0)
        {
          while(digitalRead(XAxisLimitSwitch2) != 0)
          {
            digitalWrite(XGantryStepper2Pulse, HIGH);
            digitalWrite(XGantryStepper2Pulse, LOW);
      
            delayMicroseconds(x_gantry_step_time);
          }
        }
        else if(digitalRead(XAxisLimitSwitch2) == 0)
        {
          while(digitalRead(XAxisLimitSwitch1) != 0)
          {
            digitalWrite(XGantryStepper1Pulse, HIGH);
            digitalWrite(XGantryStepper1Pulse, LOW);
      
            delayMicroseconds(x_gantry_step_time);
          }
        }
        // Conduct the X Gantry Calibration Sequence
        XGantryCalibrationSequence();
      
        // Set the gantry_positions.x_gantry_step_count to 0
        gantry_positions.x_gantry_step_count = 0;*/
      
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }        
      if(gantry_positions.x_gantry_step_count == 0)
      {
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable X Gantry Steppers
        digitalWrite(XGantryStepper1Enable, HIGH);
        digitalWrite(XGantryStepper2Enable, HIGH);
        return;
      }
      digitalWrite(XGantryStepper1Pulse, HIGH);
      digitalWrite(XGantryStepper2Pulse, HIGH);
      digitalWrite(XGantryStepper1Pulse, LOW);
      digitalWrite(XGantryStepper2Pulse, LOW);
      gantry_positions.x_gantry_step_count--;

      delayMicroseconds(x_gantry_step_time);
    }
  }

  gantry_positions.x_gantry_position = ((float)(gantry_positions.x_gantry_step_count) / x_gantry_steps_per_revolution) * x_gantry_distance_per_revolution;

  EEPROM_writeAnything(0, gantry_positions);
  
  if(abs(num_x_gantry_steps) <= x_gantry_step_interval)
  {
    move_x_gantry_flag = false;
    // Disable X Gantry Steppers
    digitalWrite(XGantryStepper1Enable, HIGH);
    digitalWrite(XGantryStepper2Enable, HIGH);
  }
}
#endif

#ifdef CONTINUOUS
void moveYGantry()
{
  // Enable Y Gantry Stepper
  digitalWrite(YGantryStepperEnable, LOW);
  
  long num_y_gantry_steps = (long)(((desired_y_gantry_position - gantry_positions.y_gantry_position) / y_gantry_distance_per_revolution) * y_gantry_steps_per_revolution);

  if(num_y_gantry_steps > 0)
  {
    // Move Y Gantry Forward
    digitalWrite(YGantryStepperDirection, LOW);

    for (long i = 0; i < num_y_gantry_steps; i++)
    {         
      if(digitalRead(YAxisLimitSwitch) == 0)
      {         
        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }
      if(gantry_positions.y_gantry_step_count >= max_y_gantry_steps)
      {
        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(YGantryStepperPulse, HIGH);
      digitalWrite(YGantryStepperPulse, LOW);
      gantry_positions.y_gantry_step_count++;

      delayMicroseconds(y_gantry_step_time);
    }
  }
  else
  {
    // Move Y Gantry Back
    digitalWrite(YGantryStepperDirection, HIGH);

    for (long i = 0; i < -num_y_gantry_steps; i++)
    { 
      if(digitalRead(YAxisLimitSwitch) == 0)
      {         
        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }        
      if(gantry_positions.y_gantry_step_count == 0)
      {
        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(YGantryStepperPulse, HIGH);
      digitalWrite(YGantryStepperPulse, LOW);
      gantry_positions.y_gantry_step_count--;

      delayMicroseconds(y_gantry_step_time);
    }
  }
  gantry_positions.y_gantry_position = ((float)(gantry_positions.y_gantry_step_count) / y_gantry_steps_per_revolution) * y_gantry_distance_per_revolution;
      
  EEPROM_writeAnything(0, gantry_positions);
  
  move_y_gantry_flag = false;
  // Disable Y Gantry Stepper
  digitalWrite(YGantryStepperEnable, HIGH);
}
#else
void moveYGantry()
{
  // Enable Y Gantry Stepper
  digitalWrite(YGantryStepperEnable, LOW);
  
  long num_y_gantry_steps = (long)(((desired_y_gantry_position - gantry_positions.y_gantry_position) / y_gantry_distance_per_revolution) * y_gantry_steps_per_revolution);

  if(num_y_gantry_steps > 0)
  {
    // Move Y Gantry Forward
    digitalWrite(YGantryStepperDirection, LOW);

    for (long i = 0; i < min(y_gantry_step_interval, num_y_gantry_steps); i++)
    {         
      if(digitalRead(YAxisLimitSwitch) == 0)
      {         
/*        // Conduct the Y Gantry Calibration Sequence
        YGantryCalibrationSequence();
    
        // Set the gantry_positions.y_gantry_step_count to 0
        gantry_positions.y_gantry_step_count = 0;*/

        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }
      if(gantry_positions.y_gantry_step_count >= max_y_gantry_steps)
      {
        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(YGantryStepperPulse, HIGH);
      digitalWrite(YGantryStepperPulse, LOW);
      gantry_positions.y_gantry_step_count++;

      delayMicroseconds(y_gantry_step_time);
    }
  }
  else
  {
    // Move Y Gantry Back
    digitalWrite(YGantryStepperDirection, HIGH);

    for (long i = 0; i < min(y_gantry_step_interval, -num_y_gantry_steps); i++)
    { 
      if(digitalRead(YAxisLimitSwitch) == 0)
      {         
/*        // Conduct the Y Gantry Calibration Sequence
        YGantryCalibrationSequence();
    
        // Set the gantry_positions.y_gantry_step_count to 0
        gantry_positions.y_gantry_step_count = 0;*/

        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }        
      if(gantry_positions.y_gantry_step_count == 0)
      {
        move_y_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Y Gantry Stepper
        digitalWrite(YGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(YGantryStepperPulse, HIGH);
      digitalWrite(YGantryStepperPulse, LOW);
      gantry_positions.y_gantry_step_count--;

      delayMicroseconds(y_gantry_step_time);
    }
  }
  gantry_positions.y_gantry_position = ((float)(gantry_positions.y_gantry_step_count) / y_gantry_steps_per_revolution) * y_gantry_distance_per_revolution;
      
  EEPROM_writeAnything(0, gantry_positions);
  
  if(abs(num_y_gantry_steps) <= y_gantry_step_interval)
  {
    move_y_gantry_flag = false;
    // Disable Y Gantry Stepper
    digitalWrite(YGantryStepperEnable, HIGH);
  }
}
#endif

#ifdef CONTINUOUS
void moveZGantry()
{
  // Enable Z Gantry Stepper
  digitalWrite(ZGantryStepperEnable, LOW);
  
  long num_z_gantry_steps = (long)(((desired_z_gantry_position - gantry_positions.z_gantry_position) / z_gantry_distance_per_revolution) * z_gantry_steps_per_revolution);

  if(num_z_gantry_steps > 0)
  {
    // Move Z Gantry Down
    digitalWrite(ZGantryStepperDirection, LOW);

    for (long i = 0; i < num_z_gantry_steps; i++)
    { 
      // Check to see if the Z Axis Limit Switch was hit
      if(digitalRead(ZAxisLimitSwitch) == 0)
      {         
        // Conduct the Z Gantry Calibration Sequence
        ZGantryCalibrationSequence();
        
        // Set the gantry_positions.z_gantry_step_count to 0
        gantry_positions.z_gantry_step_count = 0;

        gantry_positions.z_gantry_position = 0.0;

        EEPROM_writeAnything(0, gantry_positions);

        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }        
      if(gantry_positions.z_gantry_step_count >= max_z_gantry_steps)
      {
        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(ZGantryStepperPulse, HIGH);
      digitalWrite(ZGantryStepperPulse, LOW);
      gantry_positions.z_gantry_step_count++;

      delayMicroseconds(z_gantry_step_time);
    }
  }
  else
  {
    // Move Z Gantry Up
    digitalWrite(ZGantryStepperDirection, HIGH);

    for (long i = 0; i < -num_z_gantry_steps; i++)
    { 
      // Check to see if the Z Axis Limit Switch was hit
      if(digitalRead(ZAxisLimitSwitch) == 0)
      {         
        // Conduct the Z Gantry Calibration Sequence
        ZGantryCalibrationSequence();
        
        // Set the gantry_positions.z_gantry_step_count to 0
        gantry_positions.z_gantry_step_count = 0;

        gantry_positions.z_gantry_position = 0.0;

        EEPROM_writeAnything(0, gantry_positions);

        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }          
      if(gantry_positions.z_gantry_step_count == 0)
      {
        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(ZGantryStepperPulse, HIGH);
      digitalWrite(ZGantryStepperPulse, LOW);
      gantry_positions.z_gantry_step_count--;

      delayMicroseconds(z_gantry_step_time);
    }
  }
  gantry_positions.z_gantry_position = ((float)(gantry_positions.z_gantry_step_count) / z_gantry_steps_per_revolution) * z_gantry_distance_per_revolution;
      
  EEPROM_writeAnything(0, gantry_positions);
  
  move_z_gantry_flag = false;
  // Disable Z Gantry Stepper
  digitalWrite(ZGantryStepperEnable, HIGH);
}
#else
void moveZGantry()
{
  // Enable Z Gantry Stepper
  digitalWrite(ZGantryStepperEnable, LOW);
  
  long num_z_gantry_steps = (long)(((desired_z_gantry_position - gantry_positions.z_gantry_position) / z_gantry_distance_per_revolution) * z_gantry_steps_per_revolution);

  if(num_z_gantry_steps > 0)
  {
    // Move Z Gantry Down
    digitalWrite(ZGantryStepperDirection, LOW);

    for (long i = 0; i < min(z_gantry_step_interval, num_z_gantry_steps); i++)
    { 
      // Check to see if the Z Axis Limit Switch was hit
      if(digitalRead(ZAxisLimitSwitch) == 0)
      {         
        // Conduct the Z Gantry Calibration Sequence
        ZGantryCalibrationSequence();
        
        // Set the gantry_positions.z_gantry_step_count to 0
        gantry_positions.z_gantry_step_count = 0;

        gantry_positions.z_gantry_position = 0.0;

        EEPROM_writeAnything(0, gantry_positions);

        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }        
      if(gantry_positions.z_gantry_step_count >= max_z_gantry_steps)
      {
        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(ZGantryStepperPulse, HIGH);
      digitalWrite(ZGantryStepperPulse, LOW);
      gantry_positions.z_gantry_step_count++;

      delayMicroseconds(z_gantry_step_time);
    }
  }
  else
  {
    // Move Z Gantry Up
    digitalWrite(ZGantryStepperDirection, HIGH);

    for (long i = 0; i < min(z_gantry_step_interval, -num_z_gantry_steps); i++)
    { 
      // Check to see if the Z Axis Limit Switch was hit
      if(digitalRead(ZAxisLimitSwitch) == 0)
      {         
        // Conduct the Z Gantry Calibration Sequence
        ZGantryCalibrationSequence();
        
        // Set the gantry_positions.z_gantry_step_count to 0
        gantry_positions.z_gantry_step_count = 0;

        gantry_positions.z_gantry_position = 0.0;

        EEPROM_writeAnything(0, gantry_positions);

        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }          
      if(gantry_positions.z_gantry_step_count == 0)
      {
        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        // Disable Z Gantry Stepper
        digitalWrite(ZGantryStepperEnable, HIGH);
        return;
      }
      digitalWrite(ZGantryStepperPulse, HIGH);
      digitalWrite(ZGantryStepperPulse, LOW);
      gantry_positions.z_gantry_step_count--;

      delayMicroseconds(z_gantry_step_time);
    }
  }
  gantry_positions.z_gantry_position = ((float)(gantry_positions.z_gantry_step_count) / z_gantry_steps_per_revolution) * z_gantry_distance_per_revolution;
      
  EEPROM_writeAnything(0, gantry_positions);
  
  if(abs(num_z_gantry_steps) <= z_gantry_step_interval)
  {
    move_z_gantry_flag = false;
    // Disable Z Gantry Stepper
    digitalWrite(ZGantryStepperEnable, HIGH);
  }
}
#endif

void turnArmTurntable()
{
  int arm_turntable_degree_error = desired_arm_turntable_degree - current_arm_turntable_degree;

  if(arm_turntable_degree_error == 0)
  {
    turn_arm_turntable_flag = false;
  }
  else if(arm_turntable_degree_error < 0)
  {
    current_arm_turntable_degree--;
    arm_turntable_servo.write(current_arm_turntable_degree);
  }
  else
  {
    current_arm_turntable_degree++;
    arm_turntable_servo.write(current_arm_turntable_degree);
  }
}

void moveArmElbow()
{
  int arm_elbow_degree_error = desired_arm_elbow_degree - current_arm_elbow_degree;

  if(arm_elbow_degree_error == 0)
  {
    move_arm_elbow_flag = false;
  }
  else if(arm_elbow_degree_error < 0)
  {
    current_arm_elbow_degree--;
    arm_elbow_servo.write(current_arm_elbow_degree);
  }
  else
  {
    current_arm_elbow_degree++;
    arm_elbow_servo.write(current_arm_elbow_degree);
  }
}

void moveEndEffector()
{
  int end_effector_degree_error = desired_end_effector_degree - current_end_effector_degree;

  if(end_effector_degree_error == 0)
  {
    move_end_effector_flag = false;
  }
  else if(end_effector_degree_error < 0)
  {
    current_end_effector_degree--;
    end_effector_servo.write(current_end_effector_degree);
  }
  else
  {
    current_end_effector_degree++;
    end_effector_servo.write(current_end_effector_degree);
  }
}
