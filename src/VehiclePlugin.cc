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

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)

/////////////////////////////////////////////////
VehiclePlugin::VehiclePlugin() 
{
  // Initialize vehicle parameters
  this->joints.resize(4);

  this->aeroLoad = 0.1;
  this->swayForce = 10;

  this->maxSpeed = 10;
  this->frontPower = 50;
  this->rearPower = 50;
  this->wheelRadius = 0.3;
  this->maxBrake = 0.0;
  this->maxGas = 0.0;
  this->steeringRatio = 1.0;
  this->tireAngleRange = 1.0;

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
  
  //_fdm.sonar_down = 0;
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

/////////////////////////////////////////////////
void VehiclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  

  this->model = _model;
  //this->physics = this->model->GetWorld()->GetPhysicsEngine();

  this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
  if (!this->joints[0])
  {
    gzerr << "Unable to find joint: front_left\n";
    return;
  }

  this->joints[1] = this->model->GetJoint(_sdf->Get<std::string>("front_right"));
  if (!this->joints[1])
  {
    gzerr << "Unable to find joint: front_right\n";
    return;
  }

  this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
  if (!this->joints[2])
  {
    gzerr << "Unable to find joint: back_left\n";
    return;
  }


  this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));
  if (!this->joints[3])
  {
    gzerr << "Unable to find joint: back_right\n";
    return;
  }

  this->joints[0]->SetParam("suspension_erp", 0, 0.15);
  this->joints[0]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[1]->SetParam("suspension_erp", 0, 0.15);
  this->joints[1]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[2]->SetParam("suspension_erp", 0, 0.15);
  this->joints[2]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[3]->SetParam("suspension_erp", 0, 0.15);
  this->joints[3]->SetParam("suspension_cfm", 0, 0.04);

  this->gasJoint = this->model->GetJoint(_sdf->Get<std::string>("gas"));
  this->brakeJoint = this->model->GetJoint(_sdf->Get<std::string>("brake"));
  this->steeringJoint = this->model->GetJoint(_sdf->Get<std::string>("steering"));

  if (!this->gasJoint)
  {
    gzerr << "Unable to find gas joint["
          << _sdf->Get<std::string>("gas") << "]\n";
    return;
  }

  if (!this->steeringJoint)
  {
    gzerr << "Unable to find steering joint["
          << _sdf->Get<std::string>("steering") << "]\n";
    return;
  }

  if (!this->joints[0])
  {
    gzerr << "Unable to find front_left joint["
          << _sdf->GetElement("front_left") << "]\n";
    return;
  }

  if (!this->joints[1])
  {
    gzerr << "Unable to find front_right joint["
          << _sdf->GetElement("front_right") << "]\n";
    return;
  }

  if (!this->joints[2])
  {
    gzerr << "Unable to find back_left joint["
          << _sdf->GetElement("back_left") << "]\n";
    return;
  }

  if (!this->joints[3])
  {
    gzerr << "Unable to find back_right joint["
          << _sdf->GetElement("back_right") << "]\n";
    return;
  }

  this->maxSpeed = _sdf->Get<double>("max_speed");
  this->aeroLoad = _sdf->Get<double>("aero_load");
  this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
  this->frontPower = _sdf->Get<double>("front_power");
  this->rearPower = _sdf->Get<double>("rear_power");

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&VehiclePlugin::OnUpdate, this)));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // Create a publisher on the ~/physics topic
  transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");
  msgs::Physics physicsMsg;
  physicsMsg.set_type(msgs::Physics::ODE);

  // Set the step time: 2.5 ms to achieve the 400 Hz required by ArduPilot on Pixhawk
  // TODO: pass it to parametric
  physicsMsg.set_max_step_size(STEP_SIZE_FOR_ARDUPILOT);
  physicsPub->Publish(physicsMsg);



  //this->velSub = this->node->Subscribe(/*std::string("~/") +
  //   this->model->GetName() +*/ "~/rover/cmd_vel", &VehiclePlugin::OnVelMsg, this);

  

  
}

/////////////////////////////////////////////////
void VehiclePlugin::Init()
{

  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Calls every initialization method. They will return 'false' in case of
  // fatal failure.
  
  if (!init_ros_side())
      return;
  
  if (!init_ardupilot_side()) //APM side
      return;  


  this->chassis = this->joints[0]->GetParent();

  // This assumes that the largest dimension of the wheel is the diameter
  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->joints[0]->GetChild());
  math::Box bb = parent->GetBoundingBox();
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;

  // The total range the steering wheel can rotate
  double steeringRange = this->steeringJoint->GetHighStop(0).Radian() -
                         this->steeringJoint->GetLowStop(0).Radian();

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = steeringRange / this->tireAngleRange;

  // Maximum gas is the upper limit of the gas joint
  this->maxGas = this->gasJoint->GetHighStop(0).Radian();

  // Maximum brake is the upper limit of the gas joint
  this->maxBrake = this->gasJoint->GetHighStop(0).Radian();

  printf("SteeringRation[%f] MaxGas[%f]\n", this->steeringRatio, this->maxGas);

  // Starts the loop thread
  _callback_loop_thread = boost::thread( boost::bind( &VehiclePlugin::loop_thread,this ) );
}

/////////////////////////////////////////////////
void VehiclePlugin::OnUpdate()
{
  // Get the normalized gas and brake amount
  double gas = this->gasJoint->GetAngle(0).Radian() / this->maxGas;
  double brake = this->brakeJoint->GetAngle(0).Radian() / this->maxBrake;

  // A little force to push back on the pedals
  this->gasJoint->SetForce(0, -0.1);
  this->brakeJoint->SetForce(0, -0.1);

  // Get the steering angle
  double steeringAngle = this->steeringJoint->GetAngle(0).Radian();

  // Compute the angle of the front wheels.
  double wheelAngle = steeringAngle / this->steeringRatio;

  // double idleSpeed = 0.5;

  // Compute the rotational velocity of the wheels
  double jointVel = (std::max(0.0, gas-brake) * this->maxSpeed) /
                    this->wheelRadius;

  // Set velocity and max force for each wheel

  // Front-left
  this->joints[0]->SetVelocityLimit(1, -jointVel);
  this->joints[0]->SetForce(1, (gas + brake) * this->frontPower);

  // Front-right
  this->joints[1]->SetVelocityLimit(1, -jointVel);
  this->joints[1]->SetForce(1, (gas + brake) * this->frontPower);

  // Back-left
  this->joints[2]->SetVelocityLimit(1, -jointVel);
  this->joints[2]->SetForce(1, (gas + brake) * this->rearPower);

  // Back-right
  this->joints[3]->SetVelocityLimit(1, -jointVel);
  this->joints[3]->SetForce(1, (gas + brake) * this->rearPower);

  // Set the front-left wheel angle
  this->joints[0]->SetLowStop(0, wheelAngle);
  this->joints[0]->SetHighStop(0, wheelAngle);
  this->joints[0]->SetLowStop(0, wheelAngle);
  this->joints[0]->SetHighStop(0, wheelAngle);

  // Set the front-right wheel angle
  this->joints[1]->SetHighStop(0, wheelAngle);
  this->joints[1]->SetLowStop(0, wheelAngle);
  this->joints[1]->SetHighStop(0, wheelAngle);
  this->joints[1]->SetLowStop(0, wheelAngle);

  // Get the current velocity of the car
  this->velocity = this->chassis->GetWorldLinearVel();

  //  aerodynamics
  this->chassis->AddForce(
  math::Vector3(0, 0, this->aeroLoad * this->velocity.GetSquaredLength()));

  // Sway bars
  math::Vector3 bodyPoint;
  math::Vector3 hingePoint;
  math::Vector3 axis;

  for (int ix = 0; ix < 4; ++ix)
  {
    hingePoint = this->joints[ix]->GetAnchor(0);
    bodyPoint = this->joints[ix]->GetAnchor(1);

    axis = this->joints[ix]->GetGlobalAxis(0).Round();
    double displacement = (bodyPoint - hingePoint).Dot(axis);

    float amt = displacement * this->swayForce;
    if (displacement > 0)
    {
      if (amt > 15)
        amt = 15;

      math::Pose p = this->joints[ix]->GetChild()->GetWorldPose();
      this->joints[ix]->GetChild()->AddForce(axis * -amt);
      this->chassis->AddForceAtWorldPosition(axis * amt, p.pos);

      p = this->joints[ix^1]->GetChild()->GetWorldPose();
      this->joints[ix^1]->GetChild()->AddForce(axis * amt);
      this->chassis->AddForceAtWorldPosition(axis * -amt, p.pos);
    }
  }
}




/*
  Main loop of the plugin.
  Handles the whole Gazebo / Ardupilot synchronisation
*/
void VehiclePlugin::loop_thread()
{

  bool isConnectionAlive = false;

  // Checks for Ardu messages at 400 Hz
  //boost::posix_time::milliseconds thread_delay(2.5);
  boost::posix_time::milliseconds thread_delay(0.5);
  //boost::posix_time::milliseconds thread_delay(500);

  ROS_INFO( PLUGIN_LOG_PREPEND "Starting listening loop for ArduPilot messages");

  // Keep running while ROS is on
  while (_rosnode->ok()) {
    // Slow down your horses !
    boost::this_thread::sleep(thread_delay);

    if (receive_apm_input()) {
            // We have a friend !
            if (!isConnectionAlive) {
                isConnectionAlive = true;
                ROS_INFO( PLUGIN_LOG_PREPEND "Connected with Ardupilot");
            }
            
            // Advances the simulation by 1 step
            //if (!_isSimPaused) {
            //    ROS_DEBUG(PLUGIN_LOG_PREPEND "step");
            //    step_gazebo_sim();
            //    iLoopCounter++;
            //}
            
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
