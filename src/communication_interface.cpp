#include "communication_interface.hpp"
#include "config.hpp"
#include "diagnostics.hpp"
#include "thruster_interface.hpp"

LinearAcceleration_t LinearAcceleration;
AngularVelocity_t AngularVelocity;
MagneticField_t MagneticField;
OrientationRPY_t orientation_RPY;
depth_t depth_data;
BatteryState_t BatteryState;





// geometry_msgs::Vector3 linearAcceleration;
// geometry_msgs::Vector3 angularVelocity;
// geometry_msgs::Vector3 magneticField;
// geometry_msgs::Vector3 orientation_RPY;
// std_msgs::Int32MultiArray pwm_values;
// std_msgs::Float32 depth_data;
// std_msgs::Bool calibration_status;
// std_msgs::Bool dropper_control;
// ros::NodeHandle nh;



// ros::Publisher linearAccelerationPub("/sensors/linear_acceleration",
//                                      &linearAcceleration);
// ros::Publisher AngularVelocityPub("/sensors/angular_velocity",
//                                   &angularVelocity);
// ros::Publisher magneticFieldPub("/sensors/magnetic_field", &magneticField);
// ros::Publisher OrientationRPYPub("/sensors/orientation", &orientation_RPY);
// ros::Publisher DepthDataPub("/sensors/depth", &depth_data);
// ros::Subscriber<std_msgs::Int32MultiArray> PWMsub("/control/pwm", &throttleCb);
// ros::Subscriber<std_msgs::Bool> calibrationSub("/control/calibration",
//                                                &calibrationCb);
// //ros::Subscriber<std_msgs::Bool> dropperSub("/control/dropper",
//                                                //&dropperCb);
// ros::Subscriber<std_msgs::Int16> diagnosticSub("/control/led", &ledCb);

void initializeCommunication() {
//   nh.initNode();
//   nh.subscribe(PWMsub);
//   nh.subscribe(calibrationSub);
//   //nh.subscribe(dropperSub);
//   nh.subscribe(diagnosticSub);
//   nh.advertise(linearAccelerationPub);
//   nh.advertise(AngularVelocityPub);
//   nh.advertise(magneticFieldPub);
//   nh.advertise(OrientationRPYPub);
//   nh.advertise(DepthDataPub);
// }
}

void sendDepth(float depth) {
  depth_data.depth = depth;
  // DepthDataPub.publish(&depth_data);
}

void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz,
                     float mx, float my, float mz) {
  LinearAcceleration.xAc = ax * G;
  LinearAcceleration.yAc = ay * G;
  LinearAcceleration.zAc = az * G;

  AngularVelocity.gx = gx;
  AngularVelocity.gy = gy;
  AngularVelocity.gz = gz;

  MagneticField.mx = mx;
  MagneticField.my = my;
  MagneticField.mz = mz;

  // linearAccelerationPub.publish(&linearAcceleration);
  // AngularVelocityPub.publish(&angularVelocity);
  // magneticFieldPub.publish(&magneticField);
}

void sendOrientation(float roll, float pitch, float yaw) {
  orientation_RPY.roll = roll;
  orientation_RPY.pitch = pitch;
  orientation_RPY.yaw = yaw;
  // OrientationRPYPub.publish(&orientation_RPY);
}
void throttleCb(void *parameters) {
  // int32_t pwm_values[NUMBER_OF_THRUSTERS];
  // for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS;
  //      thruster_index++) {
  //   pwm_values[thruster_index] = pwm_msg.data[thruster_index];
  // }
  // setThrusterThrottle(pwm_values);
}

void calibrationCb(void *parameters) {
  // bool calibration_mode = calibration_status.data;
  // nh.loginfo("Calibration Mode received.");
  // callUpdateOffset(calibration_mode);
}

void ledCb(void *parameters) {
  // int16_t led_indicator = led_msg.data;
  // setLED(led_indicator);
  // nh.loginfo("Led indicator received.");

}

// void dropperCb(const std_msgs::Bool& dropper_control){
//   bool activate = dropper_control.data;
//   if(activate)
//   activateDropper();
// }

// void checkForCommands() { nh.spinOnce(); }
