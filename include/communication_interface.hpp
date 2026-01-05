#ifndef COMMUNICATION_INTERFACE_HPP
#define COMMUNICATION_INTERFACE_HPP

#include <Arduino.h>
#include "config.h"
// #include <geometry_msgs/Vector3.h>
// #include <ros.h>
// #include <sensor_msgs/MagneticField.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/Int32MultiArray.h>
//extern ros::NodeHandle nh;
// void initializeCommunication();
// void sendDepth(float depth);
// void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz,
//                      float mx, float my, float mz);
// void sendOrientation(float roll, float pitch, float yaw);
// void throttleCb(const std_msgs::Int32MultiArray& pwm_msg);
// void calibrationCb(const std_msgs::Bool& calibration_status);
// void ledCb(const std_msgs::Int16& led_msg);
// void checkForCommands();
//void dropperCb(const std_msgs::Bool& dropper_control);

typedef struct {
  // Orientation
  float roll;
  float pitch;
  float yaw;
} OrientationRPY_t;

typedef struct {
  // Linear acceleration (body frame, m/s^2)
  float xAc;
  float yAc;
  float zAc;
}LinearAcceleration_t;

typedef struct {
  // Angular velocity (rad/s)
  float gx;
  float gy;
  float gz;
} AngularVelocity_t;

typedef struct {
  // Magnetic field (uT or raw, your choice)
  float mx;
  float my;
  float mz;
} MagneticField_t;

typedef struct {
  // Depth (meters)
  float depth;
}depth_t;

typedef struct {
  // Battery
  float battery_voltage;
  float battery_current;
} BatteryState_t;
  // Timestamp (micros)

uint32_t last_update_us;

typedef struct {
  // Thruster commands (PWM or normalized, your choice)
  int16_t thruster_pwm[NUMBER_OF_THRUSTERS];

  // Mode commands
  bool calibration_mode;
  bool dropper_cmd;

  // Command freshness
  uint32_t last_rx_us;

} CommandState;


typedef enum {
  LED_OFF = 0,
  LED_READY,
  LED_WAITING_FOR_CMD,
  LED_JETSON_LOST,
  LED_BATTERY_LOW,
  LED_FAILSAFE,
} LedMode;

typedef enum {
  FAILSAFE_NONE = 0,
  FAILSAFE_COMMS_TIMEOUT,
  FAILSAFE_BATTERY_CRITICAL,
  FAILSAFE_MANUAL,
} FailsafeReason;

typedef struct {
  // Link status
  bool jetson_alive;
  bool comms_ok;

  // Battery status
  bool battery_low;
  bool battery_critical;

  // Failsafe
  bool failsafe_active;
  FailsafeReason failsafe_reason;

  // Visual feedback
  LedMode led_mode;

  // Timestamp
  uint32_t last_health_update_us;

} SystemHealth;



#endif  // COMMUNICATION_INTERFACE_HPP