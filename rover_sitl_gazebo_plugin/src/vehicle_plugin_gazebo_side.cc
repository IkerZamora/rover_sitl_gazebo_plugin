/*
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
 */

/*
 * Gazebo Plugin:  ardupilot_sitl_gazebo_plugin
 * Description: 
 *   Implements plugin's methods related to communication with Gazebo.
 *   e.g. initialization, Gazebo topics subscriptions, callback methods
 */


// FUTURE TODO:
// implement a reset, using for GUI inputs the callback 'on_gazebo_control(...)' with:
//    _msg->has_reset()
//    _msg->reset().has_all()
//    _msg->reset().has_time_only()
//    _msg->reset().has_model_only()


#include "rover_sitl_gazebo_plugin.hh"

namespace gazebo
{

//-------------------------------------------------
//  Initialization methods
//-------------------------------------------------

/*
  Initializes variables related to Gazebo.
  In case of fatal failure, returns 'false'.
 */
bool RoverSitlGazeboPlugin::init_gazebo_side(physics::WorldPtr world, sdf::ElementPtr sdf)
{
    // Saves pointers to the parent world
    _parent_world = world;
    _sdf = sdf;

    // Pause de simulation until rover model is inserted
    ROS_INFO("Waiting for rover model to be inseted...");
    _parent_world->SetPaused(true);
    while (!world->GetModel("rover")){
        continue;
    }
    ROS_INFO("Rover model inserted!");
    _rover_model = world->GetModel("rover");
    _parent_world->SetPaused(false);


    // Setup Gazebo node infrastructure
    if (_sdf->HasElement("UAV_MODEL"))
        _modelName = _sdf->Get<std::string>("UAV_MODEL");
    //if (_sdf->HasElement("NB_SERVOS_MOTOR_SPEED"))
    //   _nbMotorSpeed = _sdf->Get<int>("NB_SERVOS_MOTOR_SPEED");
    ROS_INFO("Model name:      %s", _modelName.c_str());
    //ROS_INFO("Nb motor servos: %d", _nbMotorSpeed);


    // FROM MODEL PLUGIN CONSTRUCTOR
    this->joints.resize(4);

    this->aeroLoad = 0.1;
    this->swayForce = 10;

    this->maxSpeed = 10;
    this->frontPower = 50;
    this->rearPower = 50;
    this->wheelRadius = 0.3;

    // FROM MODEL PLUGIN Load()
    this->joints[0] = _rover_model->GetJoint(_sdf->Get<std::string>("front_left"));
    if (!this->joints[0])
    {
    gzerr << "Unable to find joint: front_left\n";
    ROS_INFO("Unable to find joint: front_left");
    return false;
    }
    ROS_INFO("front_left joint found");
    this->joints[1] = _rover_model->GetJoint(
      _sdf->Get<std::string>("front_right"));

    if (!this->joints[1])
    {
    gzerr << "Unable to find joint: front_right\n";
    return false;
    }

    this->joints[2] = _rover_model->GetJoint(_sdf->Get<std::string>("back_left"));
    if (!this->joints[2])
    {
    gzerr << "Unable to find joint: back_left_joint\n";
    return false;
    }


    this->joints[3] = _rover_model->GetJoint(_sdf->Get<std::string>("back_right"));
    if (!this->joints[3])
    {
    gzerr << "Unable to find joint: back_right_joint\n";
    return false;
    }

    this->joints[0]->SetParam("suspension_erp", 0, 0.15);
    this->joints[0]->SetParam("suspension_cfm", 0, 0.04);

    this->joints[1]->SetParam("suspension_erp", 0, 0.15);
    this->joints[1]->SetParam("suspension_cfm", 0, 0.04);

    this->joints[2]->SetParam("suspension_erp", 0, 0.15);
    this->joints[2]->SetParam("suspension_cfm", 0, 0.04);

    this->joints[3]->SetParam("suspension_erp", 0, 0.15);
    this->joints[3]->SetParam("suspension_cfm", 0, 0.04);

    this->gasJoint = _rover_model->GetJoint(_sdf->Get<std::string>("gas"));
    this->brakeJoint = _rover_model->GetJoint(_sdf->Get<std::string>("brake"));
    this->steeringJoint = _rover_model->GetJoint(
      _sdf->Get<std::string>("steering"));

    if (!this->gasJoint)
    {
    gzerr << "Unable to find gas joint["
          << _sdf->Get<std::string>("gas") << "]\n";
    return false;
    }

    if (!this->steeringJoint)
    {
    gzerr << "Unable to find steering joint["
          << _sdf->Get<std::string>("steering") << "]\n";
    return false;
    }

    if (!this->joints[0])
    {
    gzerr << "Unable to find front_left joint["
          << _sdf->GetElement("front_left") << "]\n";
    return false;
    }

    if (!this->joints[1])
    {
    gzerr << "Unable to find front_right joint["
          << _sdf->GetElement("front_right") << "]\n";
    return false;
    }

    if (!this->joints[2])
    {
    gzerr << "Unable to find back_left joint["
          << _sdf->GetElement("back_left") << "]\n";
    return false;
    }

    if (!this->joints[3])
    {
    gzerr << "Unable to find back_right joint["
          << _sdf->GetElement("back_right") << "]\n";
    return false;
    }


    this->maxSpeed = _sdf->Get<double>("max_speed");
    this->aeroLoad = _sdf->Get<double>("aero_load");
    this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
    this->frontPower = _sdf->Get<double>("front_power");
    this->rearPower = _sdf->Get<double>("rear_power");

    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RoverSitlGazeboPlugin::on_gazebo_update, this)));

    this->node = transport::NodePtr(new transport::Node());

    this->node->Init(_rover_model->GetWorld()->GetName());

    int argc = 0;
    ros::init(argc, NULL, "rover_model_plugin");

    //ros::NodeHandle rosnode;
    //this->velSub = _rosnode.subscribe("/rover/cmd_vel", 100, &RoverSitlGazeboPlugin::OnVelMsg, this);

    // FROM MODEL PLUGIN Init()

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

    ROS_INFO("SteeringRatio[%f] MaxGas[%f]\n", this->steeringRatio, this->maxGas);

    //-------------------------------------------------


    // 'transport' is the communication library of Gazebo. It handles publishers
    // and subscribers.
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(_parent_world->GetName());
    
    _parent_world->SetPaused(true);
    
    // Create a publisher on the ~/physics topic
    transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::ODE);

    // Set the step time: 2.5 ms to achieve the 400 Hz required by ArduPilot on Pixhawk
    // TODO: pass it to parametric
    physicsMsg.set_max_step_size(STEP_SIZE_FOR_ARDUPILOT);
    physicsPub->Publish(physicsMsg);
    
    _controlSub = node->Subscribe("~/world_control", &RoverSitlGazeboPlugin::on_gazebo_control, this);
    
    //_modelInfoSub = node->Subscribe("~/model/info", &RoverSitlGazeboPlugin::on_gazebo_modelInfo, this);
    
    //this->newFrameConnection = this->camera->ConnectNewImageFrame(
    //  boost::bind(&CameraPlugin::OnNewFrame, this, _1, _2, _3, _4, _5));

    _updateConnection = event::Events::ConnectWorldUpdateEnd(
          boost::bind(&RoverSitlGazeboPlugin::on_gazebo_update, this));
    // Or we could also use 'ConnectWorldUpdateBegin'
    // For a list of all available connection events, see: Gazebo-X.X/gazebo/common/Events.hh 
    
    ROS_INFO("Gazebo side initialized");
    return true;
}

    
//-------------------------------------------------
//  Gazebo communication
//-------------------------------------------------

/*
  Advances the simulation by 1 step
 */
void RoverSitlGazeboPlugin::step_gazebo_sim()
{
    // The simulation must be in Pause mode for the Step function to work.
    // This ensures that Ardupilot does not miss a single step of the physics solver.
    // Unfortunately, it breaks the Gazebo system of Real Time clock and Factor,
    // as well as the functionnality of the Pause & Step GUI buttons.
    // The functionnality of the Pause GUI button is emulated within 'on_gazebo_control()'.
    _parent_world->Step(1);
}

/*
  Callback from gazebo after each simulation step
  (thus after each call to 'step_gazebo_sim()')
 */
void RoverSitlGazeboPlugin::on_gazebo_update()
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. USe mutexes if required.
    
    // Get the simulation time
    gazebo::common::Time gz_time_now = _parent_world->GetSimTime();
    // Converts it to seconds
    _fdm.timestamp = gz_time_now.sec + gz_time_now.nsec * 1e-9;

    if (!_timeMsgAlreadyDisplayed) {
        // (It seems) The displayed value is only updated after the first iteration
        ROS_INFO( PLUGIN_LOG_PREPEND "Simulation step size is = %f", _parent_world->GetPhysicsEngine()->GetMaxStepSize());
        _timeMsgAlreadyDisplayed = true;
    }

    // MODEL PLUGIN STUFF

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
    this->joints[0]->SetVelocityLimit(1, -jointVel);
    this->joints[0]->SetForce(1, (gas + brake) * this->frontPower);

    this->joints[1]->SetVelocityLimit(1, -jointVel);
    this->joints[1]->SetForce(1, (gas + brake) * this->frontPower);

    this->joints[2]->SetVelocityLimit(1, -jointVel);
    this->joints[2]->SetForce(1, (gas + brake) * this->rearPower);

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
  Emulates the Pause GUI button functionnality.
  Shortcomings: The GUI button does not change shape between Play/Resume
 */
void RoverSitlGazeboPlugin::on_gazebo_control(ConstWorldControlPtr &_msg)
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    if (_msg->has_pause()) {
        // The plugin's running principle, based on explicit calls to Gazebo's step() method,
        // requires Gazebo to be in a continuous pause state.
        // Indeed, Gazebo must permanently remain in pause, until the next call to step().
        // Therefore the state of the GUI play/pause button is not reliable, and the
        // actual play/pause state must be handled here, with '_isSimPaused'.
        
        _isSimPaused = !_isSimPaused;
        
        if (_isSimPaused) {
            _parent_world->SetPaused(true);
            ROS_INFO( PLUGIN_LOG_PREPEND "Simulation is now paused");
        } else {
            ROS_INFO( PLUGIN_LOG_PREPEND "Resuming simulation");
        }
    }
}

/*
  Callback method when a new model is added on Gazebo.
  Used to detect the end of asynchronous model loading.
 */
/*
void RoverSitlGazeboPlugin::on_gazebo_modelInfo(ConstModelPtr &_msg)
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    ROS_DEBUG( PLUGIN_LOG_PREPEND "on_gazebo_modelInfo, name %s", _msg->name().c_str());
    
    if (!_msg->name().compare(PARACHUTE_MODEL_NAME)) {
        // Gazebo has finally finished loading the parachute model.
        // It's about time, our UAV was falling to the ground at high speed !!!
        on_parachute_model_loaded();
    }
}
*/



} // end of "namespace gazebo"
