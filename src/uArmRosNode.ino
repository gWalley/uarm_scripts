// uArmRosNode.ino
// Arduino Sketch written to act as a ROS Node for the uArm.

// Libraries
#include <ArduinoHardware.h>
#include <ros.h>

// Standard messages used
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

// EEPROM for saving calibration data
#include <EEPROM.h>
// UF_uArm contains standard control functionality for the Arm.
#include <UF_uArm.h>

// Declare a UF_uArm object to call control functions
UF_uArm uarm_robot;

int16_t positions[4] = {0};
// Keeps track of previous value of button presses
bool b_d4_last = 0;
bool b_d7_last = 0;
bool b_ls_last = 0;
/* ===================================================================================== */
// ROS Subscription Callback Functions
/* ===================================================================================== */

// jointCallBack: Commands to set position of robot arm are dealt with by this callback
void jointCallback(const std_msgs::Int16MultiArray &cmd) {
  uarm_robot.setPosition(cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3]);
}
// stretch, height, arm_rot, hand_rot

// gripperCallback: commands to enable and disable the pump are handled by this callback
void gripperCallback(const std_msgs::Bool &cmd) {
    if (cmd.data == 1){
        uarm_robot.gripperCatch();
    }else if(cmd.data == 0){
        uarm_robot.gripperRelease();
    }
}

// gripperDetachCallback: commands to manually detached gripper are handled by this callback
void gripperDetachCallback(const std_msgs::Bool &cmd){
    if (cmd.data == 1){
        uarm_robot.gripperDirectDetach();
    }
}
/* ===================================================================================== */
// ROS Subscription declarations
/* ===================================================================================== */
// Declare node
ros::NodeHandle nh;
// Declare Subscriptions
//  - Gripper commands
ros::Subscriber<std_msgs::Bool> g_sub("/uarm/gripper", gripperCallback);
//  - Joint movement commands
ros::Subscriber<std_msgs::Int16MultiArray> j_sub("/uarm/joint_commands", jointCallback);
//  - Gripper detach commands (disable valve to prevent it from drawing excess power)
ros::Subscriber<std_msgs::Bool> g_d_sub("/uarm/gripper_detach", gripperDetachCallback);

/* ===================================================================================== */
// ROS Publisher Declarations
/* ===================================================================================== */
// Declare Publications
//  - Publishing current servo angles
std_msgs::Int16MultiArray joint_state_msg;
ros::Publisher joint_state_pub("/uarm/joint_states", &joint_state_msg);
//  - Publishing button presses
std_msgs::Bool b_d4_msg;
ros::Publisher b_d4_pub("/uarm/b/d4", &b_d4_msg);
std_msgs::Bool b_d7_msg;
ros::Publisher b_d7_pub("/uarm/b/d7", &b_d7_msg);
std_msgs::Bool b_ls_msg;
ros::Publisher b_ls_pub("/uarm/b/ls", &b_ls_msg);

// Initialisation function called at start-up and on reset
void setup()
{
    // Initialise node
    nh.initNode();
    // Publishers are initialised with the advertise function
    nh.advertise(joint_state_pub);
    nh.advertise(b_d4_pub);
    nh.advertise(b_d7_pub);
    nh.advertise(b_ls_pub);
    // Subscriptions are initialised with the subscribe function
    nh.subscribe(g_sub);
    nh.subscribe(j_sub);
    nh.subscribe(g_d_sub);
    
    // Initialise uArm
    uarm_robot.init();
    uarm_robot.setServoSpeed(SERVO_R,        0); // 0=full speed, 1-255 slower to faster
    uarm_robot.setServoSpeed(SERVO_L,        0);
    uarm_robot.setServoSpeed(SERVO_ROT,      0);
    uarm_robot.setServoSpeed(SERVO_HAND_ROT, 0);
    joint_state_msg.data_length = 4;
    joint_state_msg.data = positions;
  
}

void loop()
{
    // Read and publish joint states
    joint_state_msg.data[0] = uarm_robot.readAngle(SERVO_R);
    joint_state_msg.data[1] = uarm_robot.readAngle(SERVO_L);
    joint_state_msg.data[2] = uarm_robot.readAngle(SERVO_ROT);
    joint_state_msg.data[3] = uarm_robot.readAngle(SERVO_HAND_ROT);
    joint_state_pub.publish(&joint_state_msg);

    // Read buttons on arm, publish if different to before
    b_d4_msg.data = (digitalRead(BTN_D4) == 0);
    b_d7_msg.data = (digitalRead(BTN_D7) == 0);
    b_ls_msg.data = (digitalRead(LIMIT_SW) == 0);

    if (b_d4_msg.data != b_d4_last){
        b_d4_pub.publish(&b_d4_msg);
        b_d4_last = b_d4_msg.data;
    }
    if (b_d7_msg.data != b_d7_last){
        b_d7_pub.publish(&b_d7_msg);
        b_d7_last = b_d7_msg.data;
    }
    if (b_ls_msg.data != b_ls_last){
        b_ls_pub.publish(&b_ls_msg);
        b_ls_last = b_ls_msg.data;
    }
    nh.spinOnce();
    // Called every loop, as after 300000 loops, gripper will auto detach.
    uarm_robot.gripperDetach();
    // Run at 50Hz
    delay(20);
}
