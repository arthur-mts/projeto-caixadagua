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
#include <freertos/queue.h>
#include <esp_intr_alloc.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "ds18b20.h" //Include library
#define DS_GPIO 3    // GPIO where you connected ds18b20
#define TRIGGER_PIN 0
#define ECHO_PIN 1
#define TAG "CAIXA DAGUA"

// Distance
struct HcSR04GetDist
{
  int isWorking;
  float distance;
};
typedef struct HcSR04GetDist HcSR04GetDist;
HcSR04GetDist distData = { 0, 0 };
QueueHandle_t gpio_evt_queue_distance = NULL;

uint64_t start_echo_time_check = 0;
uint64_t end_echo_time_check = 0;

// Temperature
Ds18b20GetTemp tempData;


void measure_distance(void *pvParameters) {
  while(1) {
    gpio_set_level(TRIGGER_PIN, 1);
    ets_delay_us(10);
    gpio_set_level(TRIGGER_PIN, 0);
    // ESP_LOGI(TAG, "Mandei o trigger");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void measure_temp(void *pvParameters)
{
  ds18b20_init(DS_GPIO);
  while (1)
  {
    tempData = ds18b20_get_temp();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void IRAM_ATTR gpio_isr_handler_distance(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue_distance, &gpio_num, NULL);
} 

void handle_echo_change(void* arg) {
  uint32_t gpio_num;
  while (1) {
    if (xQueueReceive(gpio_evt_queue_distance, &gpio_num, portMAX_DELAY)) {
      int gpio_level = gpio_get_level(gpio_num);


      if (gpio_level == 0) {
        end_echo_time_check = esp_timer_get_time();
        ESP_LOGI(TAG, "Borda de descida");
        float time_diff = (float) end_echo_time_check - start_echo_time_check;
        float distance = time_diff / 58.0;
        distData.isWorking = 1;
        distData.distance = distance;
        ESP_LOGI(TAG, "Distancia: status=%i; value=%.2f", distData.isWorking, distData.distance);
      } else {
        start_echo_time_check = esp_timer_get_time();
        ESP_LOGI(TAG, "Borda de subida");
      }
    }
  }
}

void config_measure_distance() {
  gpio_pad_select_gpio(TRIGGER_PIN);
  gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);


  gpio_pad_select_gpio(ECHO_PIN);
  gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
  gpio_set_intr_type(ECHO_PIN, GPIO_INTR_ANYEDGE);
  
  gpio_evt_queue_distance = xQueueCreate(10, sizeof(uint32_t));

  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
  gpio_isr_handler_add(ECHO_PIN, gpio_isr_handler_distance, (void*)ECHO_PIN);

  xTaskCreate(&measure_distance, "measure_distance", 2048, NULL, 5, NULL);
  xTaskCreate(&handle_echo_change, "handle_echo_change", 2048, NULL, 1, NULL);
}

void app_main() {
  config_measure_distance();
  xTaskCreate(&measure_temp, "measure_temp", 1024, NULL, 5, NULL);

  // while(1) {
  //   ESP_LOGI(TAG, "Distancia: status=%i; value=%.2f", distData.isWorking, distData.distance);
  //   // ESP_LOGI(TAG, "Temperatura: status=%i; value=%.2f", tempData.isWorking, tempData.temp);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
}