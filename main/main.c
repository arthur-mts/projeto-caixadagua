/*
   This is example for my DS18B20 library
   https://github.com/feelfreelinux/ds18b20


   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "ds18b20.h" //Include library
#define DS_GPIO 3    // GPIO where you connected ds18b20
#define TRIGGER_PIN 0
#define ECHO_PIN 1
#define TAG "CAIXA DAGUA"

struct HcSR04GetDist
{
  int isWorking;
  float distance;
};
typedef struct HcSR04GetDist HcSR04GetDist;

Ds18b20GetTemp tempData;
HcSR04GetDist distData = { 0, 0 };


void measure_distance(void *pvParameters) {
  while(1) {
    gpio_set_level(TRIGGER_PIN, 1);
    ets_delay_us(10);
    gpio_set_level(TRIGGER_PIN, 0);

    uint64_t start_time_check = esp_timer_get_time();
    distData.isWorking = 1;

    while(gpio_get_level(ECHO_PIN) == 0) {
      if((esp_timer_get_time() - start_time_check) >= 500) {
        distData.isWorking = 0;
        goto continue_parent_loop;
      }
    }

    uint64_t start_time = esp_timer_get_time();
    while(gpio_get_level(ECHO_PIN) == 1) {
      if((esp_timer_get_time() - start_time_check) >= 500) {
        distData.isWorking = 0;
        goto continue_parent_loop;
      }
    }

    uint64_t end_time = esp_timer_get_time();

    float time_diff = (float) end_time - start_time;
    float distance = time_diff / 58.0;
    distData.isWorking = 1;
    distData.distance = distance;
    continue_parent_loop:
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void measure_temp(void *pvParameters)
{
  ds18b20_init(DS_GPIO);
  while (1)
  {

    tempData = ds18b20_get_temp();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void app_main() {
  ESP_LOGI(TAG, "Iniciando");
  gpio_pad_select_gpio(TRIGGER_PIN);
  gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT );
  gpio_pad_select_gpio(ECHO_PIN);
  gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

  // o esp ta morrendo qnd da um log de erro
  xTaskCreate(&measure_distance, "measure_distance", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  // xTaskCreate(&measure_temp, "measure_temp", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

  vTaskStartScheduler();
  while(1) {
    ESP_LOGI(TAG, "Distancia: status=%i; value=%i", distData.isWorking, distData.distance);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}