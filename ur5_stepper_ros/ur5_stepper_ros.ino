//Transforming the motor's rotary motion into linear motion by using a threaded rod:
//Threaded rod's pitch = 5 mm. This means that one revolution will move the nut 2 mm.
//Default stepping = 400 step/revolution.
// 400 step = 1 revolution = 5 mm linear motion. (4 start 2 mm pitch screw)
// 1mm = 80 steps

//character for commands

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <AccelStepper.h>
#include <std_srvs/Empty.h>
#include <std_srvs/position_prune.h>
#include <rosserial_arduino/Test.h>
//#include <ur5_pruning_trails/position.h>

// ---------------------------------------------------------------------------
//            // Setting stepper
//  -------------------------------------------------------------------
int directionMultiplier = 1, next_dir =1; // = 1: positive direction, = -1: negative direction
bool newData, runallowed = false; // booleans for new data from serial, and runallowed flag
AccelStepper stepper(1, 8, 9);// direction Digital 9 (CCW), pulses Digital 8 (CLK)
long motor_steps =0, move_steps = 0; //Number of step
int motor_speed = 1500;
int motor_acc= 100;
long receivedAcceleration = 0; //Steps / second^2
char receivedCommand;
// ---------------------------------------------------------------------------
//            // Setting Limit Switch
//  -------------------------------------------------------------------
long limit_switchA = 1, limit_switchB = 1, limit_switch = 1; // default is 1
int limit_switch_pinA = 3, limit_switch_pinB = 2;
long leader_dist = 250, leader_steps=0, steps = 0,  calib=0;
int lead = 1, pitch =5, microstep =400;
int scaling_factor = microstep/(pitch* lead);
int dist_to_step, step_to_dist;
int limit_count=1, lc=0;

// ---------------------------------------------------------------------------
            // Setting ROS
 // -------------------------------------------------------------------
// Create a service server that takes nothing and returns nothing
ros::NodeHandle  nh;
using rosserial_arduino::Test;

bool homing = false, move_7thaxis= false;

void Homing()
{
  while (true)
  { motor_steps = 2000;

    if (directionMultiplier == 1)
    {
      limit_switch = digitalRead(limit_switch_pinA);
  
    }
    else
    {
      limit_switch = digitalRead(limit_switch_pinB);
    }
   
    if (limit_switch != 0)
    { RotateRelative() ;
    }
    else
    { Stop();
      delay(100);
      
      int i =0;

      UpdateHome();
//      while(i<1000)
//      {motor_steps =1000;
//      directionMultiplier = -1*directionMultiplier;
//      i++;
//      RotateRelative
//      ();} 
      stepper.moveTo(1000);
      UpdateHome();
      delay(100);
      homing= false;
      break;
    }
  }
}


void RunTheMotor() //function for the motor
{
  if (runallowed == true)
  { //Serial.println("move");
      nh.spinOnce();
    delay(1);
    stepper.enableOutputs(); //enable pins
    stepper.run(); //step the motor (this will step the motor by 1 step at each loop)
  }
  else //program enters this part if the runallowed is FALSE, we do not do anything
  {
    stepper.disableOutputs(); //disable outputs
    return;
  }
}

int ConvertDistToStep(int dist)
{ dist_to_step = dist * scaling_factor;
  return dist_to_step;
}

//int ConvertStepToDist(int steps)
//{ step_to_dist = steps / scaling_factor;
//  return step_to_dist;
//}




void calibrate_leader_dist()
{ 
}

  void GoHome()
  { runallowed = true;
    if (stepper.currentPosition() == 0)
    {

      stepper.disableOutputs(); //disable power
    }
    else
    {
      stepper.setMaxSpeed(400); //set speed manually to 400. In this project 400 is 400 step/sec = 1 rev/sec.
      stepper.moveTo(0); //set abolute distance to move
    }
    RunTheMotor(); //function to handle the motor
  }

  void RotateRelative()
  {

    runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepper.setMaxSpeed(motor_speed); //set speed
    stepper.move(directionMultiplier * motor_steps); //set relative distance and direction
    RunTheMotor(); //function to handle the motor

  }

  void RotateAbsolute()
  {

    if (directionMultiplier == 1)
    {
      limit_switch = digitalRead(limit_switch_pinA);
  
    }
    else
    {
      limit_switch = digitalRead(limit_switch_pinB);
    }
   
    if (limit_switch != 0)
    {runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepper.setMaxSpeed(motor_speed); //set speed
    
    stepper.moveTo(move_steps); //set relative distance
    RunTheMotor(); //function to handle the motor
    }
    else
    { Stop();
      delay(100);
    }

  }
  void Stop()
  {
    stepper.stop(); //stop motor
    stepper.disableOutputs(); //disable powe
    runallowed = false;


  }
//
//    void Stop_lm()
// {
//
//  static unsigned long interrupttime = 0;
//  if (millis() - interrupttime > 100) {
//      stepper.stop();
//    interrupttime = millis();
//  }
//
// }
//  

  void UpdateHome()
  {
    runallowed = false; //we still keep running disabled
    stepper.disableOutputs(); //disable power
    stepper.setCurrentPosition(0); //Reset current position. "new home"
    RunTheMotor(); //function to handle the motor

  }

//  void limitStopA()
//  { 
//    runallowed = false;
//    Stop_lm();
//    }
//
//
//  
//  void limitStopB()
//  { 
//    runallowed = false;
//     Stop_lm();
//    }  
//    
void callback_home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // Simulate function running for a non-deterministic amount of time
//  delay(random(5,70));
   digitalWrite(13, HIGH);
   digitalWrite(11, HIGH);
   directionMultiplier = -1;
   nh.loginfo("Start homing");
   homing=true;  
}

void callback_move(const Test::Request & req, Test::Response &res)
{  
//   leader_steps = ConvertDistToStep(leader_dist);
//  leader_steps = req;
   nh.loginfo("moving leader");
   nh.loginfo(req.input);
   move_7thaxis=true;  
   move_steps = (leader_steps + move_steps);
   directionMultiplier = 1;
   if (move_steps > 60000)
      move_steps=60000;
}


ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> server_home("move_home", &callback_home);
ros::ServiceServer<Test::Request, Test::Response> server_move("move_linear", &callback_move);
std_msgs::Float32 str_msg;
ros::Publisher position_("linear_position", &str_msg);

float linear_position = 0.0;

void setup()
{
  nh.initNode();
  nh.advertiseService(server_home);
  nh.advertiseService(server_move);
  nh.advertise(position_);
// ---------------------------------------------------------------------------
//            // Setting Pins
//  -------------------------------------------------------------------

  stepper.setMaxSpeed(motor_speed); //SPEED = Steps / second
  stepper.setAcceleration(motor_acc); //ACCELERATION = Steps /(second)^2
  stepper.disableOutputs(); //disable outputs
//   ---------------------------------------------------------------------------
//            // Limit switch
//  -------------------------------------------------------------------
  
  pinMode(limit_switch_pinA, INPUT); //LimitSwitch_1 (default is 1 for me)
  pinMode(limit_switch_pinB, INPUT);
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);




}

void loop()

{   
  if (homing== true)
    Homing();
  else if(move_7thaxis == true)
    {  stepper.setMaxSpeed(motor_speed);
    RotateAbsolute();
  }
  linear_position = stepper.currentPosition();
  str_msg.data = linear_position;
  position_.publish(&str_msg); 
  nh.spinOnce();
  delay(1);
  


 
}
