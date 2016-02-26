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
 *   Implements plugin's methods related to the communication with
 *   the SITL interface of Ardupilot.
 *   e.g. covers ports opening, unpacking/packing input/output data.
 */

#include "VehiclePlugin.hh"

namespace gazebo
{

//-------------------------------------------------
//  Initialization methods
//-------------------------------------------------

/*
  Initializes variables and ports related to ArduPilot.
  In case of fatal failure, returns 'false'.
 */
bool VehiclePlugin::init_ardupilot_side()
{
    // Setup network infrastructure (opens ports from/to ArduPilot)
    open_control_socket();
    open_fdm_socket();

    ROS_INFO( PLUGIN_LOG_PREPEND "APM/Ardupilot side initialized !"); 

    return true;
}


//-------------------------------------------------
//  ArduPilot communication - initialization
//-------------------------------------------------

/*
  open control socket from ArduPilot
 */
bool VehiclePlugin::open_control_socket()
{
    if (_is_control_socket_open)
        return true;

    ROS_INFO( PLUGIN_LOG_PREPEND "Binding to listening port from ArduPilot...\n");
    if (!_sock_control_from_ardu->bind("127.0.0.1", PORT_DATA_FROM_ARDUPILOT)) {
        ROS_WARN( PLUGIN_LOG_PREPEND "FAILED to bind to port from ArduPilot\n");
        return false;
    }

    ROS_INFO( PLUGIN_LOG_PREPEND "SUCCESS in binding to port from ArduPilot\n");
    _sock_control_from_ardu->set_blocking(false);
    //_sock_control_from_ardu->reuseaddress();
    _is_control_socket_open = true;

    return true;
}

/*
  open fdm socket to ArduPilot
 */
bool VehiclePlugin::open_fdm_socket()
{
    if (_is_fdm_socket_open)
        return true;

    ROS_INFO( PLUGIN_LOG_PREPEND "Connecting send port to ArduPilot...\n");
    if (!_sock_fdm_to_ardu->connect("127.0.0.1", PORT_DATA_TO_ARDUPILOT)) {
        //check_stdout();
        ROS_WARN( PLUGIN_LOG_PREPEND "FAILED to connect to port to ArduPilot\n");
        return false;
    }

    ROS_INFO( PLUGIN_LOG_PREPEND "Opened ArduPilot fdm socket\n");
    _sock_fdm_to_ardu->set_blocking(false);
    _is_fdm_socket_open = true;

    // First message: introduction
    // (may not be received if ArduPilot is not yet running)
    char startup[] = "";
    _sock_fdm_to_ardu->send(startup, strlen(startup));
    return true;
}

    
//-------------------------------------------------
//  ArduPilot communication - running
//-------------------------------------------------

/*
  Receive control inputs from the APM SITL and publishes them in a command/motor_speed topic
 */
bool VehiclePlugin::receive_apm_input()
{
    servo_packet pkt;
    ssize_t szRecv;

    if (!_is_control_socket_open) {
        ROS_INFO( PLUGIN_LOG_PREPEND "Cannot receive input from Ardu, for the port is not open !");
        return false;
    }
    ROS_INFO("first");
    szRecv = _sock_control_from_ardu->recv(&pkt, sizeof(pkt), 100);
    ROS_INFO("second");
    // Expects a servo control packet
    if (szRecv != sizeof(servo_packet)) {
        return false;
    }

    // Overwrites the servo control message
    int i;

    // Convert from values from 0 to 1, to values from -50 to 50
    for (i=0; i<NB_SERVOS_MOTOR_SPEED; i++) {
        _cmd_motor_speed[i] = 100.0 * (pkt.servos[i] - 0.5);
    }

    for (i=0; i<NB_SERVOS_MOTOR_SPEED; i++) {
        ROS_INFO("motor[%d] = %f",i,_cmd_motor_speed[i]);
    }
    publish_commandMotorSpeed();
    return true;
}


/*
  Packages the fdmState data and sends it to the APM SITL
 */
void VehiclePlugin::send_apm_output()
{
    fdm_packet pkt;

    if (!_is_control_socket_open) {
        ROS_INFO( PLUGIN_LOG_PREPEND "Cannot send output to Ardu, for the port is not open !");
        return;
    }   
    
    // Mutex on '_fdm', for it is concurrently written by ROS callbacks
    _fdm_mutex.lock();
    ROS_INFO("here?");
    memcpy(&pkt, &_fdm, sizeof(fdm_packet));
    ROS_INFO("nope");
    _fdm_mutex.unlock();
    
    // Makes sure the timestamp is non 0, otherwise Ardupilot can believe it to be an erroneous packet
    if (pkt.timestamp < 1e-6)
        pkt.timestamp = 1e-6;       // 1e-6 [s] = 0.001 [ms]
    ROS_INFO("maybe here?");
    ssize_t sent = _sock_fdm_to_ardu->send(&pkt, sizeof(pkt));
    ROS_INFO("neither");
}

} // end of "namespace gazebo"
