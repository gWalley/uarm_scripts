The files contained in this project allow the uFactory uArm to be used with ROS, to carry out coordinated tasks with other robots

Useful Files:
  - ./src/uArmRosNode.ino  :Arduino sketch, upload to the uArm. (Buffer must be increased from 64 Bytes to at least 512 Bytes)
  - ./src/main_1_5.py      :The main script, run by using "rosrun uarm_scripts main_1_5.py"
  - ./src/ArmNode_1_5.py   :Contains state machine and functions used to control the Arm.
  - ./src/FSM.py           :Class Structure definitions of Finite State Machine
  - ./launch/uarm.launch   :ROS launch file to initiate nodes required to run all scripts
                           - Launches usb_cam, CMvision, and rosserial_python nodes
                           - CMvision package has been modified slightly to change the name of the blobs topic to /uarm/blobs
                             as multiple copies of CMvision were being run on same laptop, for both the base and the arm.


Package depends on:
  - usb_cam: Ros package to access raw camera footage from a usb webcam
  - CMvision: Coloured blob detection in images
  - rosserial: Allows devices connected via Serial to interface with ROS
  - robot_comm: Package created by jkprice07, client code for communication with server used in this project