#include "main.hpp"
#include "Wire.h"

float ax, ay, az, gx, gy, gz, mx, my, mz, depth, roll, pitch, yaw;

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(ACCEL_ID, MAG_ID);
MPU6050 gyro;
MS5837 depth_sensor;

// long int previous_update_rate = 0, previous_publish_rate = 0;

void publishTask(void * parameter) {
  while(1){
      sendIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
      sendOrientation(roll, pitch, yaw);
      sendDepth(depth);      
    
    vTaskDelay(pdMS_TO_TICKS(500)); 
  }
}

void updateTask(void * parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  while(1){
    updateIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
    updateDepthSensorReadings(depth);
    applyIMUCalibration(ax, ay, az, gx, gy, gz, mx, my, mz);
    updateOrientation(ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw);
    //readVoltage();

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
  }
}

void setup() {
  // Serial.begin(9600);
  // while (!Serial) {
  //   delay(1);
  // }

  Wire.begin();
  
  initializeCommunication();
  initializeIMU();
  initializeDepthSensor();
  initializeSensorMath();
  initializeThrusters();
  initializeLED();
  //initializeDropper();
  // previous_update_rate = micros();
  // previous_publish_rate = millis();

  xTaskCreatePinnedToCore(
    updateTask,          /* Task function. */
    "Update Task",      /* name of task. */
    4096,             /* Stack size of task */
    NULL,              /* parameter of the task */
    3,                 /* priority of the task */
    NULL,              /* Task handle to keep track of created task */
    1);                /* pin task to core 1 */

  xTaskCreatePinnedToCore(
    publishTask,          /* Task function. */
    "Publish Task",      /* name of task. */
    3072,             /* Stack size of task */
    NULL,              /* parameter of the task */
    1,                 /* priority of the task */
    NULL,              /* Task handle to keep track of created task */
    1);                /* pin task to core 1 */
}

void loop() {
  // if ((micros() - previous_update_rate) >= UPDATE_RATE)
  // {
  //   previous_update_rate = micros();
  //   updateIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
  //   updateDepthSensorReadings(depth);
  //   applyIMUCalibration(ax, ay, az, gx, gy, gz, mx, my, mz);
  //   updateOrientation(ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw);
  //   //readVoltage();
  // }

  // if ((millis() - previous_publish_rate) >= PUBLISH_RATE) {
  //   previous_publish_rate = millis();
  //   sendIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
  //   sendOrientation(roll, pitch, yaw);
  //   sendDepth(depth);

  //   Serial.print("roll: "); Serial.println(roll);
  //   Serial.print("pitch: "); Serial.println(pitch);
  //   Serial.print("yaw: "); Serial.println(yaw);
    
  // }
  // checkForCommands();
}
