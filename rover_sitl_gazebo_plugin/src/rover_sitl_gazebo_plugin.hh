/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: A 4-wheeled vehicle
 * Author: Nate Koenig
 */

#ifndef __GAZEBO_VEHICLE_PLUGIN_HH__
#define __GAZEBO_VEHICLE_PLUGIN_HH__


//Standard includes
#include <string>
#include <vector>
#include <thread>

//Gazebo includes
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

//ROS includes
#include <ros/ros.h>
#include <ros/console.h>

//ROS messages
#include <geometry_msgs/Twist.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>              // for the sonar
#include <sensor_msgs/NavSatFix.h>          // for the GPS fix
#include <geometry_msgs/Vector3Stamped.h>   // for the GPS velocity fix
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

//Plugin's inner headers
#include "SocketAPM.hh"

// Plugin's services
#include "rover_sitl_gazebo_plugin/TakeApmLapseLock.h"
#include "rover_sitl_gazebo_plugin/ReleaseApmLapseLock.h"

// This plugin implements a thread, based on boost, for the communication with Ardupilot
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// Network
#include <sys/socket.h>
#include <netinet/in.h>

//----------------------------------------
// Sensors configuration
#define ENABLED     1
#define DISABLED    0

#define SONAR_FRONT     ENABLED

//----------------------------------------
// Model description name
#define MODEL_NAME  "rover"

// All ROS topics emitted by this plugin with have the
// namespace "/fdmUDP" prepended.
#define ROS_NAMESPACE "/fdmUDP"

//----------------------------------------- 
// Communication with Ardupilot

#define PORT_DATA_FROM_ARDUPILOT       9002
#define PORT_DATA_TO_ARDUPILOT         9003

// Messages passed
#define NB_SERVOS               16
#define NB_SERVOS_MOTOR_SPEED   4

#define STEERING_SERVO          0
#define THROTTLE_SERVO          2

#define STEP_SIZE_FOR_ARDUPILOT     0.0025      // in [s], = 400 Hz


#define MAX_LAPSE_LOCK_ON_MODEL_INSERT   5     // [s]
#define MAX_LAPSE_LOCK_DEFAULT           1     // [s]

//--------------------------------------------
// Math
#define PI      3.1415926536
#define PI_2    1.5707963268

//--------------------------------------------
// Log/Debug
#define PLUGIN_LOG_PREPEND       "ROVER: "

// Uncomment the following lines to activate some debug
//#define DEBUG_DISP_GPS_POSITION
#define DEBUG_DISP_MOTORS_COMMANDS


namespace gazebo
{

class ArduPilotSitlGazeboPlugin : public WorldPlugin
{

  public:
    /// \brief Constructor
    ArduPilotSitlGazeboPlugin();

    ~ArduPilotSitlGazeboPlugin();

    //Public function members
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
    //virtual void Init();

    // MAIN LOOP related methods ---------------
    void take_lapseLock(float max_holder_lock_duration = MAX_LAPSE_LOCK_DEFAULT);
    bool release_lapseLock();
    
    // GAZEBO related methods ------------------
    void on_gazebo_update();
    void on_gazebo_control(ConstWorldControlPtr &_msg);
    void on_gazebo_modelInfo(ConstModelPtr &_msg);
  
    // ROS related methods ---------------------
    void imu_callback(const sensor_msgs::Imu &imu_msg);
    void gps_callback(const sensor_msgs::NavSatFix &gps_fix_msg);
    void gps_velocity_callback(const geometry_msgs::Vector3Stamped &gps_velocity_fix_msg);
    void sonar_down_callback(const sensor_msgs::Range &sonar_range_msg);
  #if SONAR_FRONT == ENABLED
    void sonar_front_callback(const sensor_msgs::Range &sonar_range_msg);
  #endif

    //void OnVelMsg(const geometry_msgs::Twist &vel_cmd);
    
    // Services:
    bool service_take_lapseLock(rover_sitl_gazebo_plugin::TakeApmLapseLock::Request  &req, rover_sitl_gazebo_plugin::TakeApmLapseLock::Response &res);
    bool service_release_lapseLock(rover_sitl_gazebo_plugin::ReleaseApmLapseLock::Request  &req, rover_sitl_gazebo_plugin::ReleaseApmLapseLock::Response &res);

  protected:

    /*
      packet sent to ardupilot_sitl_gazebo
     */
    struct servo_packet {
        float servos[NB_SERVOS];    // ranges from 0 (no rotation) to 1 (full throttle)
    };

    /*
      reply packet sent from Gazebo to ArduPilot
     */
    struct fdm_packet {
      double timestamp;                             // [seconds] simulation time
      double imu_angular_velocity_rpy[3];           // [rad/s]
      double imu_linear_acceleration_xyz[3];        // [m/s/s] in NED, body frame
      double imu_orientation_quat[4];               // rotation quaternion, APM conventions, from body to earth
      double velocity_xyz[3];                       // [m/s] in NED
      double position_xyz[3];                       // [m] in NED, from Gazebo's map origin (0,0,0)
      double position_latlonalt[3];                 // [degrees], altitude is Up

      // You can add here extra sensors to pass along
      double sonar_down;                            // [m] downward facing range finder
      
    #if SONAR_FRONT == ENABLED
      double sonar_front;                           // [m] forward facing range finder
    #endif
      
    };

    // Initialization methods ------------------
    bool init_ros_side();
    bool init_gazebo_side(physics::WorldPtr world, sdf::ElementPtr sdf);
    bool init_ardupilot_side();

    // MAIN LOOP related methods ---------------
    void loop_thread();
    bool check_lapseLock(float loop_elapsed_dt);
    void clear_lapseLock();


    // callback methods
    void OnUpdate();
    void OnVelMsg(const geometry_msgs::Twist vel_cmd/*ConstPosePtr &_msg*/);


    // ARDUPILOT related methods
    bool open_control_socket();
    bool open_fdm_socket();
    bool receive_apm_input();
    void send_apm_output();

    
    // GAZEBO related methods ------------------
    void step_gazebo_sim();

    // ROS related methods ---------------------
    void quat_to_euler(float q1, float q2, float q3, float q4,
                       float &roll, float &pitch, float &yaw);
    void publish_commandMotorSpeed();


    // Connection data
    std::vector<event::ConnectionPtr> connections;

    SocketAPM *_sock_fdm_to_ardu;
    SocketAPM *_sock_control_from_ardu;

    bool _is_control_socket_open;
    bool _is_fdm_socket_open;

    fdm_packet                  _fdm;
    boost::mutex                _fdm_mutex;
    
    boost::thread               _callback_loop_thread;

    //GAZEBO data
    gazebo::physics::WorldPtr   _parent_world;
    physics::ModelPtr _rover_model;
    physics::LinkPtr chassis;
    std::vector<physics::JointPtr> joints;
    physics::JointPtr gasJoint, brakeJoint;
    physics::JointPtr steeringJoint;

    math::Vector3 velocity;

    ros::Subscriber velSub;

    double frontPower, rearPower;
    double maxSpeed;
    double wheelRadius;

    double steeringRatio;
    double tireAngleRange;
    double maxGas, maxBrake;

    double aeroLoad;
    double swayForce;
    //physics::LinkPtr chassis;
    //std::vector<physics::JointPtr> joints;
    //physics::JointPtr gasJoint, brakeJoint;
    //physics::JointPtr steeringJoint;
    sdf::ElementPtr             _sdf;
    transport::SubscriberPtr    _controlSub;      // Subscriber used to receive updates on world_control topic.
    transport::SubscriberPtr    _modelInfoSub;

    //GAZEBO transport data
    transport::NodePtr node;
    //transport::SubscriberPtr velSub;

    bool                        _isSimPaused;     // Flag to hold the simulation (pause)
    bool                        _timeMsgAlreadyDisplayed;  // for debug of the time step

    //ROS messages
    ros::NodeHandle* _rosnode;
    //---
    //ros::Subscriber cmd_vel_sub; //remove?
    //---
    ros::ServiceServer          _service_take_lapseLock;
    ros::ServiceServer          _service_release_lapseLock;
    
    ros::Subscriber             _imu_subscriber;
    ros::Subscriber             _sonar_down_subscriber;
    ros::Subscriber             _sonar_front_subscriber;
    ros::Subscriber             _gps_subscriber;
    ros::Subscriber             _gps_velocity_subscriber;
    
    
    ros::Publisher              _motorSpd_publisher;

    float                       _cmd_motor_speed[NB_SERVOS_MOTOR_SPEED];    // Local copy of the motor speed command, in [rad/s]


    // Vehicle parameters
    std::string                 _modelName;
    int                         _nbMotorSpeed;
    
    // Timing
    ros::Duration               _control_period;
    ros::Time                   _last_update_sim_time_ros;
    ros::Time                   _last_write_sim_time_ros;
    
    event::ConnectionPtr        _updateConnection;


    float                       _loop_lapseLock;            // [s] blocks the main loop until release (-> 0), or if the timer is elapsed
    int                         _nbHolders_lapseLock;       // [1] number of calling processes to the lapselock (case of multiple lockers)
    boost::mutex                _lapseLock_mutex;           // to protect the lapse-lock from simulatenous modifications

};

} // end of namespace gazebo

#endif // GAZEBO_VEHICLE_PLUGIN_HH
