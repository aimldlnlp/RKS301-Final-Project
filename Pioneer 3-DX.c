#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SPEED 5.24
#define MAX_SENSOR_NUMBER 16
#define MAX_SENSOR_VALUE 1024
#define MIN_DISTANCE 1.0  // Jarak minimal untuk deteksi tembok (meter)

typedef enum { FORWARD, LEFT, RIGHT } State;

static WbDeviceTag sensors[MAX_SENSOR_NUMBER];

int main() {
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag left_wheel = wb_robot_get_device("left wheel");
  WbDeviceTag right_wheel = wb_robot_get_device("right wheel");

  // Inisialisasi sensor jarak
  char sensor_name[5] = "";
  for (int i = 0; i < MAX_SENSOR_NUMBER; ++i) {
    sprintf(sensor_name, "so%d", i);
    sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }

  // Inisialisasi roda
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

  double speed[2] = {0.0, 0.0};
  State state = FORWARD;

  while (wb_robot_step(time_step) != -1) {
    // Membaca nilai sensor dan menghitung jarak rata-rata
    double left_distance = 0.0;
    double right_distance = 0.0;
    int left_count = 0, right_count = 0;

    for (int i = 0; i < MAX_SENSOR_NUMBER; ++i) {
      double sensor_value = wb_distance_sensor_get_value(sensors[i]);

      // Konversi nilai sensor ke jarak (dalam meter)
      double distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));

      // Sensor kiri (0-3)
      if (i >= 0 && i <= 3) {
        left_distance += distance;
        left_count++;
      }
      // Sensor kanan (4-7)
      else if (i >= 4 && i <= 7) {
        right_distance += distance;
        right_count++;
      }
    }

    // Rata-rata jarak untuk kiri dan kanan
    if (left_count > 0) left_distance /= left_count;
    if (right_count > 0) right_distance /= right_count;

    // Logika keputusan
    if (left_distance < MIN_DISTANCE || right_distance < MIN_DISTANCE) {
      if (left_distance > right_distance) {
        state = LEFT;  // Menjauh dari tembok kanan
      } else {
        state = RIGHT;  // Menjauh dari tembok kiri
      }
    } else {
      state = FORWARD;  // Tidak ada tembok -> maju
    }

    // Atur kecepatan berdasarkan state
    switch (state) {
      case FORWARD:
        speed[0] = MAX_SPEED;  // Roda kiri
        speed[1] = MAX_SPEED;  // Roda kanan
        break;
      case LEFT:
        speed[0] = -0.5 * MAX_SPEED;  // Roda kiri mundur
        speed[1] = 0.5 * MAX_SPEED;   // Roda kanan maju
        break;
      case RIGHT:
        speed[0] = 0.5 * MAX_SPEED;   // Roda kiri maju
        speed[1] = -0.5 * MAX_SPEED;  // Roda kanan mundur
        break;
    }

    // Set kecepatan roda
    wb_motor_set_velocity(left_wheel, speed[0]);
    wb_motor_set_velocity(right_wheel, speed[1]);
  }

  wb_robot_cleanup();
  return 0;
}
