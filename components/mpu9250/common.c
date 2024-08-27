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

#include "common.h"

/*
这类似于一个while循环的帧率控制函数：保持该函数在“不大于1000/SAMPLE_INTERVAL_MS的帧率运行”
*/
void pause(void)
{

  static uint64_t start = 0; // 改变量在函数调用结束后，不会消失，一直存在
  // configTICK_RATE_HZ:每秒tick个数
  // xTaskGetTickCount: 获取开机以来，当前tick总数
  // end：开机到现在的毫秒数
  uint64_t end = xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;

  if (start == 0) // 首次调用
  {
    start = xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;
  }

  int32_t elapsed = end - start;
  if (elapsed < SAMPLE_INTERVAL_MS) // SAMPLE_INTERVAL_MS=5ms
  {
    vTaskDelay((SAMPLE_INTERVAL_MS - elapsed) / portTICK_PERIOD_MS); // portTICK_PERIOD_MS=1000ms
  }
  start = xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;
}
