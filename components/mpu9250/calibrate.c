/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "mpu9250.h"
#include "common.h"

const char *TAG = "calibrate";

// Default calibration is to have none.
static calibration_t cal = {
    .mag_offset = {.x = 0.0, .y = 0.0, .z = 0.0},
    .mag_scale = {.x = 1.0, .y = 1.0, .z = 1.0},
    .accel_offset = {.x = 0.0, .y = 0.0, .z = 0.0},
    .accel_scale_lo = {.x = -1.0, .y = -1.0, .z = -1.0},
    .accel_scale_hi = {.x = 1.0, .y = 1.0, .z = 1.0},
    .gyro_bias_offset = {.x = 0.0, .y = 0.0, .z = 0.0}};

void wait(void)  //delay 10秒
{
  for (int i = 10; i >= 0; i -= 1)
  {
    printf("Starting in %d seconds     \r", i);
    fflush(stdout);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  printf("\n");
}

static void init_imu(void)
{
  static bool init_imu_done = false;  // 仅初始化一次
  if (init_imu_done)
    return;
  i2c_mpu9250_init(&cal);
  init_imu_done = true;
}

/**
 * 
 * GYROSCOPE
 * 
 * 
 * Calibrate the gyro.  The device needs to remain still during calibration.  The calibration will
 * be applied to the gyro.  This is only simple calibration for Gyro bias for when the Gyro is still.
 * More sophisticated calibration tools can be applied.
 *
 * NOTE: The Gyro must not be moved during this process.
 *
 */

const int NUM_GYRO_READS = 5000;

void calibrate_gyro(void)
{
  init_imu();

  ESP_LOGI(TAG, "--- GYRO CALIBRATION ---");
  ESP_LOGW(TAG, "Keep the MPU very still.  Calculating gyroscope bias");
  wait(); // 10秒睡眠

  vector_t vg_sum;
  vg_sum.x = 0.0;
  vg_sum.y = 0.0;
  vg_sum.z = 0.0;
  for (int i = 0; i < NUM_GYRO_READS; i += 1)  //多次读取
  {

    vector_t vg;

    ESP_ERROR_CHECK(get_gyro(&vg));

    vg_sum.x += vg.x;
    vg_sum.y += vg.y;
    vg_sum.z += vg.z;

    // Make the WDT happy
    if (i % 100 == 0)
      vTaskDelay(0);

    pause();
  }

  vg_sum.x /= -NUM_GYRO_READS;  // 为什么要加上 负号呢
  vg_sum.y /= -NUM_GYRO_READS;
  vg_sum.z /= -NUM_GYRO_READS;

  printf("    .gyro_bias_offset = {.x = %f, .y = %f, .z = %f}\n", vg_sum.x, vg_sum.y, vg_sum.z);
}

/**
 * 
 * ACCELEROMETER 
 * 
 * 
 * Calibrate the Accelerometer.  This device will need to be rotated[翻转] with the X, Y and Z axes up and down.  The axis
 * you point up/down will be calibrated against[针对/根据] gravity[重力] (so you must have it vertical[垂直]).  
 * You may want to hold it against[根据] a wall[墙] or similar.  
 * While the one axis is being calibrated against gravity, the other two axes will be perpendicular[垂直]
 * to gravity, so will read near[取近] zero, this value will be used as the offset(这时这个值就是偏移量).
 *
 * The scaling is simple linear scaling[线性缩放], based on the common formular[公式] for a line, y = m * x + c, 
 * where y is our scaled[缩放系数] and offset[偏移] result, while x is the raw value.  
 * This formular is actually applied in the main mpu9250.js file.  But
 * this calibration process outputs those values.
 */

#define NUM_ACCEL_READS (1000.0)

#define X_AXIS (0)
#define Y_AXIS (1)
#define Z_AXIS (2)
const char *axes[] = {"X", "Y", "Z"};

#define DIR_UP (0)
#define DIR_DOWN (1)
const char *directions[] = {"up", "down"};

vector_t offset = {.x = 0, .y = 0, .z = 0};
vector_t scale_lo = {.x = 0, .y = 0, .z = 0};
vector_t scale_hi = {.x = 0, .y = 0, .z = 0};

/**
 * This will syncronuously read the accel data from MPU9250.  It will gather the offset and scalar values.
 */
void calibrate_accel_axis(int axis, int dir)
{
  vector_t va;

  ESP_LOGI(TAG, "Reading values - hold still");
  for (int i = 0; i < NUM_ACCEL_READS; i++)  // 1000次
  {
    get_accel(&va);

    if (axis == X_AXIS)
    {
      if (dir == DIR_UP)  // X-up
      {
        scale_lo.x += va.x;
      }
      else  // X-down
      {
        scale_hi.x += va.x;
      }
    }
    else
    {
      offset.y += va.y; // 计算g的水平投影y
      offset.z += va.z; // 计算g的水平投影z
    }

    if (axis == Y_AXIS)
    {
      if (dir == DIR_UP)
      {
        scale_lo.y += va.y; // y的up方向累加
      }
      else
      {
        scale_hi.y += va.y; // y的down方向累加
      }
    }
    else
    {
      offset.x += va.x;
      offset.z += va.z;
    }

    if (axis == Z_AXIS)
    {
      if (dir == DIR_UP)
      {
        scale_lo.z += va.z;
      }
      else
      {
        scale_hi.z += va.z;
      }
    }
    else
    {
      offset.x += va.x;
      offset.y += va.y;
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

/**
 * Set up the next capture for an axis and a direction (up / down).
 */
void run_next_capture(int axis, int dir)
{
  ESP_LOGW(TAG, "Point the %s axis arrow %s.", axes[axis], directions[dir]);

  wait();  // 延时10秒
  calibrate_accel_axis(axis, dir);
}

void calibrate_accel(void)  //校准acc
{
  init_imu();

  ESP_LOGI(TAG, "--- ACCEL CALIBRATION ---");

  // 跟随进行操作
  run_next_capture(X_AXIS, DIR_UP);
  run_next_capture(X_AXIS, DIR_DOWN);
  run_next_capture(Y_AXIS, DIR_UP);
  run_next_capture(Y_AXIS, DIR_DOWN);
  run_next_capture(Z_AXIS, DIR_UP);
  run_next_capture(Z_AXIS, DIR_DOWN);

  // 计算
  offset.x /= (NUM_ACCEL_READS * 4.0); // 每个多计算了4倍
  offset.y /= (NUM_ACCEL_READS * 4.0);
  offset.z /= (NUM_ACCEL_READS * 4.0);
  scale_lo.x /= NUM_ACCEL_READS;
  scale_lo.y /= NUM_ACCEL_READS;
  scale_lo.z /= NUM_ACCEL_READS;
  scale_hi.x /= NUM_ACCEL_READS;
  scale_hi.y /= NUM_ACCEL_READS;
  scale_hi.z /= NUM_ACCEL_READS;
  // 校准结果
  printf("    .accel_offset = {.x = %f, .y = %f, .z = %f},\n    .accel_scale_lo = {.x = %f, .y = %f, .z = %f},\n    .accel_scale_hi = {.x = %f, .y = %f, .z = %f},\n",
         offset.x, offset.y, offset.z,
         scale_lo.x, scale_lo.y, scale_lo.z,
         scale_hi.x, scale_hi.y, scale_hi.z);
}

/**
 *
 * MAGNETOMETER
 * 
 * 
 * Once the calibration is started you will want to move the sensor around all axes[8字一路狂甩].  What we want is to find the
 * extremes[极值] (min/max) of the x, y, z values such that[以便] we can find the offset and scale values.
 *
 * These calibration calculations are based on this page:
 * http://www.camelsoftware.com/2016/03/13/imu-maths-calculate-orientation-pt3/
 */

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)

void calibrate_mag(void)
{

  vector_t v_min = {
      .x = 9.9e99,
      .y = 9.9e99,
      .z = 9.9e99};
  vector_t v_max = {
      .x = -9.9e99,
      .y = -9.9e99,
      .z = -9.9e99};

  const int NUM_MAG_READS = 2000;

  init_imu();

  ESP_LOGW(TAG, "Rotate the magnometer around all 3 axes, until the min and max values don't change anymore.");
  wait();  // 延时10秒,等待用户准备

  printf("    x        y        z      min x     min y     min z     max x     max y     max z\n");
  for (int i = 0; i < NUM_MAG_READS; i += 1)
  {
    vector_t vm;
    get_mag(&vm);
    v_min.x = MIN(v_min.x, vm.x);
    v_min.y = MIN(v_min.y, vm.y);
    v_min.z = MIN(v_min.z, vm.z);
    v_max.x = MAX(v_max.x, vm.x);
    v_max.y = MAX(v_max.y, vm.y);
    v_max.z = MAX(v_max.z, vm.z);

    printf(" %0.3f    %0.3f    %0.3f    %0.3f   %0.3f   %0.3f   %0.3f   %0.3f   %0.3f       \r", vm.x, vm.y, vm.z, v_min.x, v_min.y, v_min.z, v_max.x, v_max.y, v_max.z);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vector_t v_avg = {
      .x = (v_max.x - v_min.x) / 2.0,
      .y = (v_max.y - v_min.y) / 2.0,
      .z = (v_max.z - v_min.z) / 2.0};
  float avg_radius = (v_avg.x + v_avg.y + v_avg.z) / 3.0;
  vector_t v_scale = {
      .x = avg_radius / v_avg.x,
      .y = avg_radius / v_avg.y,
      .z = avg_radius / v_avg.z};

  printf("\n");
  printf("    .mag_offset = {.x = %f, .y = %f, .z = %f},\n", (v_min.x + v_max.x) / 2, (v_min.y + v_max.y) / 2, (v_min.z + v_max.z) / 2);
  printf("    .mag_scale = {.x = %f, .y = %f, .z = %f},\n", v_scale.x, v_scale.y, v_scale.z);
}
