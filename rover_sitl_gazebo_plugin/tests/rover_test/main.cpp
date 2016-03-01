#include "SocketAPM.hh"

#include <iostream>

#define NB_SERVOS               16

/*
  packet sent to ardupilot_sitl_gazebo
 */
struct servo_packet {
    float servos[NB_SERVOS];    // ranges from 0 (no rotation) to 1 (full throttle)
};
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

 };


int main(int argc, char* argv[])
{
    fdm_packet _fdm;
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
    _fdm.imu_orientation_quat[3] = 0;
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
    _fdm.position_xyz[2] = 0;    // [m]

    SocketAPM* socketapm = new SocketAPM(true);
    if (!socketapm->bind("127.0.0.1", 9002)) {
        printf("FAILED to connect to port to ArduPilot\n");
        return false;
    }
    socketapm->set_blocking(false);
    socketapm->reuseaddress();

    servo_packet pkt;
    ssize_t szRecv;

    SocketAPM* socketapm2 = new SocketAPM(true);
    if (!socketapm2->connect("127.0.0.1", 9003)) {
        printf("FAILED to connect to port to ArduPilot\n");
        return false;
    }
    socketapm2->set_blocking(false);
    char startup[] = "";
    ssize_t sent = socketapm2->send(startup, strlen(startup));
    std::cout << " sent : " << sent<< std::endl;

    while(1){
        szRecv = socketapm->recv(&pkt, sizeof(pkt), 100);
        // Expects a servo control packet
        if (szRecv != sizeof(servo_packet)) {
            std::cout << " Error receiving: " << szRecv<< std::endl;
        }else{
            std::cout << " szRecv : "<< szRecv << std::endl;
        }
        fdm_packet pkt_fdm;

        // Mutex on '_fdm', for it is concurrently written by ROS callbacks
        memcpy(&pkt_fdm, &_fdm, sizeof(fdm_packet));

        // Makes sure the timestamp is non 0, otherwise Ardupilot can believe it to be an erroneous packet
        pkt_fdm.timestamp = 1e-6;       // 1e-6 [s] = 0.001 [ms]

        ssize_t sent2 = socketapm2->send(&pkt_fdm, sizeof(pkt_fdm));
        std::cout << " sent2 : " << sent2<< std::endl;
        sleep(1);
    }


}
