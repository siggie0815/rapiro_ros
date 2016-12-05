#include <ros.h>
#include <rapiro_msgs/JointState.h>
#include <rapiro_msgs/Ranging.h>
#include <rapiro_msgs/Trajectory.h>

#include <Servo.h>
#include <Ultrasonic.h>

#include "rapiro.h"

#define SHIFT 7

// global trajectory variables
byte cur_pose[N_SERVOS];
byte tgt_pose[N_SERVOS];
int delta[N_SERVOS];
unsigned long tgt_time[N_SERVOS];

// ROS messages
rapiro_msgs::JointState state_msg;
char usr_frame[] = "usr_frame";

// loop frequency
unsigned long next_traj, next_pub;
const unsigned long per_traj =  10; // period [ms] => 100Hz
const unsigned long per_pub  = 100; // period [ms] => 10Hz

// actuators and sensors
Servo servo[N_SERVOS];
Ultrasonic sensor(PIN_TRIG, PIN_ECHO);

// callback functions
void trajectory_cb(const rapiro_msgs::Trajectory&);
void ranging_srv_cb(const rapiro_msgs::Ranging::Request&, rapiro_msgs::Ranging::Response&);

// ROS
ros::NodeHandle_<ArduinoHardware, 2, 2, 100, 100> nh;
ros::Publisher pub_state("joint_state", &state_msg);
ros::Subscriber<rapiro_msgs::Trajectory> sub_trajectory("trajectory_cmd", &trajectory_cb);
ros::ServiceServer<rapiro_msgs::Ranging::Request, rapiro_msgs::Ranging::Response> srv_ranging("ranging", &ranging_srv_cb);

// init controller
void setup()
{
  // set up sensor
  sensor.Config();
  
  // set up and start all servos
  servo[0].attach(PIN_HEAD);      // Head yaw
  servo[1].attach(PIN_WAIST);     // Waist yaw
  servo[2].attach(PIN_R_SHLD_R);  // R Sholder roll
  servo[3].attach(PIN_R_SHLD_P);  // R Sholder pitch
  servo[4].attach(PIN_R_HAND);    // R Hand grip
  servo[5].attach(PIN_L_SHLD_R);  // L Sholder roll
  servo[6].attach(PIN_L_SHLD_P);  // L Sholder pitch
  servo[7].attach(PIN_L_HAND);    // L Hand grip
  servo[8].attach(PIN_R_FOOT_Y);  // R Foot yaw
  servo[9].attach(PIN_R_FOOT_P);  // R Foot pitch
  servo[10].attach(PIN_L_FOOT_Y); // L Foot yaw
  servo[11].attach(PIN_L_FOOT_P); // L Foot pitch  
  memcpy(tgt_pose, zero_pose, N_SERVOS);
  memcpy(cur_pose, zero_pose, N_SERVOS);
  for (int id = 0; id < N_SERVOS; ++id)
  {
    servo[id].write(trim_and_clamp(id));
    tgt_time[id] = 0;
  }
  pinMode(PIN_PWR, OUTPUT);
  digitalWrite(PIN_PWR, HIGH);
  
  // set up joint state message
  state_msg.pos_length = N_SERVOS;
  state_msg.pos = cur_pose;

  // wait for HC-05
  delay(1500);
  next_traj = millis();
  next_pub = millis();
  
  // start ROS Node
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_state);
  nh.advertiseService(srv_ranging);
  nh.subscribe(sub_trajectory);
}

// main loop
void loop() {

  // execute trajectories
  unsigned long now = millis();
  if (next_traj <= now)
  {
    next_traj = now + per_traj;

    for (int id = 0; id < N_SERVOS; ++id)
    {
      if (tgt_time[id] != 0)
      {
        interpolate_traj(id, now);
        servo[id].write(trim_and_clamp(id));
      }
    }
  }
  
  // publish state and do ros communication
  now = millis();
  if (next_pub <= now)
  {
    next_pub = now + per_pub;

    range_msg.header.stamp = nh.now();
    range_msg.range = float(sensor.Ranging(CM)) / 100.0;
    pub_range.publish(&range_msg);

    state_msg.header.stamp = nh.now();
    pub_state.publish(&state_msg);
    
    nh.spinOnce();
  }
  
} // end of main loop

// receive new trajectory, set target positions and compute delta per timestep
void trajectory_cb(const rapiro_msgs::Trajectory &msg)
{
  // Discard bad trajectory messages
  if (msg.tgt_pos_length  != msg.id_length ||
     (msg.tgt_time_length != msg.id_length && msg.tgt_time_length != 1) )
  {
    nh.logwarn("Bad trajectory!");
    return;
  }

  bool one_time = msg.tgt_time_length == 1;
  unsigned long now = millis();

  // goto initial pose when empty trajectory is received
  if (msg.id_length == 0)
  {
      for (int id = 0; id < N_SERVOS; ++id)
      {
        tgt_pose[id] = zero_pose[id];
        tgt_time[id] = now + (one_time ? (msg.tgt_time[0]*100) : 1000);

        int e = (tgt_pose[id] - cur_pose[id]) << SHIFT;
        int n = (tgt_time[id] - now) / per_traj;
        delta[id] = e / n;
      }
  }
  // set received target trajectory
  else for (int j = 0; j < msg.id_length; j++)
  {
    byte id = msg.id[j];
    tgt_pose[id] = msg.tgt_pos[j];
    tgt_time[id] = now + msg.tgt_time[one_time ? 0 : j] * 100;

    int e = (tgt_pose[id] - cur_pose[id]) << SHIFT;
    int n = (tgt_time[id] - now) / per_traj;
    delta[id] = e / n;
  }
}

// compute new position for current timestep
void interpolate_traj(byte id, unsigned long now)
{
  int n = (tgt_time[id] - now) / per_traj;
  
  if (n > 0)
  {
    // timesteps left ==> interpolate new position
    cur_pose[id] = tgt_pose[id] - ((n * delta[id]) >> SHIFT);
  }
  else
  {
    // no timesteps left ==> go to target and stop
    tgt_time[id] = 0;
    cur_pose[id] = tgt_pose[id];
  }
}

// validate new servo angle
byte trim_and_clamp(byte id)
{
  // apply servo offset
  int val = cur_pose[id] + trim_angle[id];
  
  // clamp into allowd range
  if (val < min_angle[id]) 
    val = min_angle[id];
    
  if (val > max_angle[id]) 
    val = max_angle[id];
  
  return (byte)val;
}

void ranging_srv_cb(const rapiro_msgs::Ranging::Request &req, rapiro_msgs::Ranging::Response &res)
{
  res.range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  res.range.header.frame_id = usr_frame;
  res.range.field_of_view = 0.367;
  res.range.min_range = 0.05;
  res.range.max_range = 4.00;
  res.range.range = float(sensor.Ranging(CM)) / 100.0;
}

