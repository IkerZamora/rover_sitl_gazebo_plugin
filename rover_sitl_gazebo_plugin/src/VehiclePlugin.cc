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

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "VehiclePlugin.hh"

//using namespace gazebo;
namespace gazebo{
GZ_REGISTER_WORLD_PLUGIN(VehiclePlugin)

/////////////////////////////////////////////////
VehiclePlugin::VehiclePlugin() 
  : WorldPlugin(),
    _is_control_socket_open(false),
    _is_fdm_socket_open(false),
    _rosnode(NULL),
    _isSimPaused(false),       // starts in running state
    _timeMsgAlreadyDisplayed(false),
    _loop_lapseLock(0.0f),
    _nbHolders_lapseLock(0)
{
  // Gazebo pointers (to world/model/joint/...) are based on Boost shared pointers.
  // To pass them to NULL, the 'reset()' method must be used.
  _parent_world.reset();
  //_uav_model.reset();
  //_parachute_model.reset();
  //_uav_chute_joint.reset();
  _rover_model.reset();

  // Initializes the FDM structure
  _fdm.timestamp = 1e-6;
  _fdm.imu_angular_velocity_rpy[0] = 0;
  _fdm.imu_angular_velocity_rpy[1] = 0;
  _fdm.imu_angular_velocity_rpy[2] = 0;
  
  _fdm.imu_linear_acceleration_xyz[0] = 0;
  _fdm.imu_linear_acceleration_xyz[1] = 0;
  _fdm.imu_linear_acceleration_xyz[2] = 0;
  
  _fdm.imu_orientation_quat[0] = 0;
  _fdm.imu_orientation_quat[1] = 0;
  _fdm.imu_orientation_quat[2] = 0;
  _fdm.imu_orientation_quat[3] = 0;
  
  _fdm.velocity_xyz[0] = 0;
  _fdm.velocity_xyz[1] = 0;
  _fdm.velocity_xyz[2] = 0;
  
  _fdm.position_xyz[0] = 0;
  _fdm.position_xyz[1] = 0;
  _fdm.position_xyz[2] = 0;
  
  _fdm.position_latlonalt[0] = 0;
  _fdm.position_latlonalt[1] = 0;
  _fdm.position_latlonalt[2] = 0;
  
  _fdm.sonar_down = 0;
#if SONAR_FRONT == ENABLED
  _fdm.sonar_front = 0;
#endif
  
  _sock_fdm_to_ardu       = new SocketAPM(true);
  _sock_control_from_ardu = new SocketAPM(true);
        
  // In case ArduPilot is a bit long to start and the '_ctrls' message is published
  // to ROS before being defined by ArduPilot. in [rad/s]
  int i;
  for (i=0; i<NB_SERVOS_MOTOR_SPEED; i++) {
      _cmd_motor_speed[i] = 0.0;
  }


}

VehiclePlugin::~VehiclePlugin()
{
  // Gazebo pointers (to world/model/joint/...) are based on Boost shared pointers.
  // To pass them to NULL, the 'reset()' method must be used.
  _parent_world.reset();
  //_uav_model.reset();
  //_parachute_model.reset();
  //_uav_chute_joint.reset();
  _rover_model.reset();
  
  // Finalizes the ROS node
  //   Make sure to call the 'shutdown' before the thread.join(),
  //   to signal the thread to stop through '_rosnode->ok()'.
  _rosnode->shutdown();
  
  // Releases all locks
  _fdm_mutex.unlock();
  _lapseLock_mutex.unlock();
  _loop_lapseLock = 0.0f;
  _nbHolders_lapseLock = 0;
  
  // Sleeps (pauses the destructor) until the thread has finished
  _callback_loop_thread.join();
  
  delete _rosnode;
  _rosnode = NULL;
}

/////////////////////////////////////////////////
void VehiclePlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  // Calls every initialization method. They will return 'false' in case of
  // fatal failure.
  
  if (!init_gazebo_side(world, sdf))
      return;
  
  if (!init_ros_side())
      return;
  
  if (!init_ardupilot_side())
      return;
  
  ROS_INFO( PLUGIN_LOG_PREPEND "Initialization finished. Every side has been initialized.");
  
  // Starts the loop thread
  _callback_loop_thread = boost::thread( boost::bind( &VehiclePlugin::loop_thread,this ) );
  
}

/*
  Main loop of the plugin.
  Handles the whole Gazebo / Ardupilot synchronisation
*/
void VehiclePlugin::loop_thread()
{

  // Or is 'boost::chrono::duration' better ?
  ros::WallTime prevloop_t_start, loop_t_start;
  ros::WallDuration loop_dt;
  bool isConnectionAlive = false;
  
  int nbSteps = 100;
  ros::WallTime prevloop10_t_start, loop10_t_start;
  ros::WallDuration loop10_dt;
  float step10_dt = STEP_SIZE_FOR_ARDUPILOT * nbSteps; // [s]

  loop10_t_start = ros::WallTime::now();
  int iLoopCounter = 0;
  
  
  // Checks for Ardu messages at 400 Hz
  //boost::posix_time::milliseconds thread_delay(2.5);
  boost::posix_time::milliseconds thread_delay(0.5);
  //boost::posix_time::milliseconds thread_delay(500);
  
  ROS_INFO( PLUGIN_LOG_PREPEND "Starting listening loop for ArduPilot messages"); 
  
  // Keeps running while ROS is on
  while (_rosnode->ok()) {


      // Slow down your horses !
      boost::this_thread::sleep(thread_delay);

      //for (int i = 0; i < 10000; i++) ROS_INFO("%d.- loop", i);

      
      // Notes the start time, calculates the loop duration
      prevloop_t_start = loop_t_start;
      loop_t_start = ros::WallTime::now();
      loop_dt = loop_t_start - prevloop_t_start;


      /*if (loop_dt.toSec() < STEP_SIZE_FOR_ARDUPILOT) {
         ROS_INFO( PLUGIN_LOG_PREPEND "Wait %f", STEP_SIZE_FOR_ARDUPILOT - loop_dt.toSec());
         boost::this_thread::sleep(boost::posix_time::seconds(STEP_SIZE_FOR_ARDUPILOT - loop_dt.toSec()));
      }*/

      //iLoopCounter++;
      if (iLoopCounter >= nbSteps) {
          iLoopCounter = 0;
          prevloop10_t_start = loop10_t_start;
          loop10_t_start = ros::WallTime::now();
          loop10_dt = loop10_t_start - prevloop10_t_start;
          if (loop10_dt.toSec() < step10_dt) {
              //ROS_INFO( PLUGIN_LOG_PREPEND "Wait %f", step10_dt - loop10_dt.toSec());
              boost::this_thread::sleep(boost::posix_time::seconds(step10_dt - loop10_dt.toSec()));
          }
      }
  

      // Checks if there is a lapse lock. If yes, waits until the other task frees it, or until the lock expires
      if (!check_lapseLock(loop_dt.toSec())) {
          continue;
      }
      

      // Checks the inbox for any email from Ardupilot
      if (receive_apm_input()) {
          // We have a friend !
          if (!isConnectionAlive) {
              isConnectionAlive = true;
              ROS_INFO( PLUGIN_LOG_PREPEND "Connected with Ardupilot");
          }

          // Advances the simulation by 1 step
          if (!_isSimPaused) {
              ROS_DEBUG(PLUGIN_LOG_PREPEND "step");
              step_gazebo_sim();
              iLoopCounter++;
          }
          // Returns the new state to ArduPilot
          send_apm_output();
      } else {
          // No message from Ardupilot on this loop, maybe next one ?
          if (isConnectionAlive) {
              isConnectionAlive = false;
              ROS_INFO( PLUGIN_LOG_PREPEND "Ardupilot connection off, no messages");
          }
      }
  }
  
  ROS_INFO( PLUGIN_LOG_PREPEND "Exited listening loop for Ardupilot messages");

}
  //-------------------------------------------------
//  LapseLock methods
//-------------------------------------------------

/*
  Service for processes so they can grab a lapse-lock for the specified maximum time (wall time).
 */
void VehiclePlugin::take_lapseLock(float max_holder_lock_duration)
{
    _lapseLock_mutex.lock();        // prevents concurrent modification of lapse-lock data
    // If the caller asks for a longer lapse-lock pause than the current one, grants him his wish
    if (_loop_lapseLock < max_holder_lock_duration) {
        _loop_lapseLock = max_holder_lock_duration;
    }
    
    // There is one more lapse-lock caller
    _nbHolders_lapseLock++;
    _lapseLock_mutex.unlock();
}

/*
  Service for processes holding a lapselock, to signal their process is finished.
  @return true if it was the last holder of the lock (= lapse-lock is now free)
 */
bool VehiclePlugin::release_lapseLock()
{
    bool is_lapseLock_now_free = false;
    
    _lapseLock_mutex.lock();        // prevents concurrent modification of lapse-lock data
    // One less lapse-lock caller
    if (_nbHolders_lapseLock > 0)
        _nbHolders_lapseLock--;
    
    // If no more holders, resets the timer
    if(!_nbHolders_lapseLock) {
        _loop_lapseLock = 0.0f;
        is_lapseLock_now_free = true;
    }
    
    _lapseLock_mutex.unlock();
    return is_lapseLock_now_free;
}


/*
  Tests if the simulation can run, or if it should wait because their is a lapse-lock.
  Decreases the lapse-lock by the specified elapsed time
  @param loop_elapsed_dt: seconds
  @return true if the simulation can run, or false if paused 
 */
bool VehiclePlugin::check_lapseLock(float loop_elapsed_dt)
{
    bool sim_can_run = false;
    
    _lapseLock_mutex.lock();        // prevents concurrent modification of lapse-lock data
    if (_loop_lapseLock < 1e-6) {
        sim_can_run = true;
    } else {
        _loop_lapseLock = _loop_lapseLock - loop_elapsed_dt;
        if (_loop_lapseLock < 0.0f) {
            // Clears the lock
            _loop_lapseLock = 0.0f;
            _nbHolders_lapseLock = 0;
            ROS_INFO( PLUGIN_LOG_PREPEND "Extern process locked the simulation for too long. Resuming the simulation...");
            sim_can_run = true;
        }
    }
    
    _lapseLock_mutex.unlock();
    return sim_can_run;
}


void VehiclePlugin::clear_lapseLock()
{
    _lapseLock_mutex.lock();        // prevents concurrent modification of lapse-lock data
    // Fully resets the lapse-lock
    _nbHolders_lapseLock = 0;
    _loop_lapseLock = 0.0f;
    _lapseLock_mutex.unlock();
}  

} // end of "namespace gazebo"
