/*
 * File:          acuatic_drone.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */

#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define LABEL_X 0.75
#define LABEL_Y 0.05
#define RED 0xFF0000

int main() {
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  
  int key;
  //const int time_step = wb_robot_get_basic_time_step();
  
  // GPS and inertial unit
  const WbDeviceTag gps = wb_robot_get_device("gps");
  const WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_gps_enable(gps, TIME_STEP);
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  
  // helixes motion
  const WbDeviceTag motor1 = wb_robot_get_device("motor_1");
  const WbDeviceTag motor2 = wb_robot_get_device("motor_2");
  const WbDeviceTag motor3 = wb_robot_get_device("motor_3");
  wb_motor_set_position(motor1, INFINITY);
  wb_motor_set_position(motor2, INFINITY);
  wb_motor_set_position(motor3, INFINITY);
  
  const WbDeviceTag sensor1 = wb_robot_get_device("D_SENSOR_1");
  const WbDeviceTag sensor2 = wb_robot_get_device("D_SENSOR_2");
  const WbDeviceTag sensor3 = wb_robot_get_device("D_SENSOR_3");
  const WbDeviceTag sensor4 = wb_robot_get_device("D_SENSOR_4");
  wb_distance_sensor_enable(sensor1, TIME_STEP);
  wb_distance_sensor_enable(sensor2, TIME_STEP);
  wb_distance_sensor_enable(sensor3, TIME_STEP);
  wb_distance_sensor_enable(sensor4, TIME_STEP);
  
  char buffer[50];
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
    key = wb_keyboard_get_key();
    
    if (key == WB_KEYBOARD_UP){
      wb_motor_set_velocity(motor1,5);
      wb_motor_set_velocity(motor2,5);
      wb_motor_set_velocity(motor3,0);
    } else if (key == WB_KEYBOARD_DOWN){
      wb_motor_set_velocity(motor1,-5);
      wb_motor_set_velocity(motor2,-5);
      wb_motor_set_velocity(motor3, 0);
    } else if (key == WB_KEYBOARD_RIGHT){
      wb_motor_set_velocity(motor1, 5);
      wb_motor_set_velocity(motor2,-5);
      wb_motor_set_velocity(motor3, 0);
    } else if (key == WB_KEYBOARD_LEFT){
      wb_motor_set_velocity(motor1,-5);
      wb_motor_set_velocity(motor2, 5);
      wb_motor_set_velocity(motor3, 0);
    } else {
      wb_motor_set_velocity(motor1,0);
      wb_motor_set_velocity(motor2,0);
      wb_motor_set_velocity(motor3,0);
    }
    
    const double altitude = wb_gps_get_values(gps)[0];
    const double yaw = wb_gps_get_values(gps)[2];
    sprintf(buffer, "X: %1.1f m", altitude);
    wb_supervisor_set_label(0, buffer, LABEL_X, LABEL_Y, 0.07, RED, 0, "Arial");
    sprintf(buffer, "Y: %1.1f m", yaw);
    wb_supervisor_set_label(1, buffer, LABEL_X, LABEL_Y -0.03, 0.07, RED, 0, "Arial");
  }
  
  return 0;
}