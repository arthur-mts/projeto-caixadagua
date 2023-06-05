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
#include "ds18b20.c" //Include library
#define DS_GPIO 3    // GPIO where you connected ds18b20
#define TRIGGER_PIN 0
#define ECHO_PIN 1

void measure_distance(void *pvParameters) {
  gpio_pad_select_gpio(TRIGGER_PIN);
  gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT );

  gpio_pad_select_gpio(ECHO_PIN);
  gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);


  while(1) {
    gpio_set_level(TRIGGER_PIN, 1);
    ets_delay_us(10);
    gpio_set_level(TRIGGER_PIN, 0);


    while(gpio_get_level(ECHO_PIN) == 0)
    ;

    uint64_t start_time = esp_timer_get_time();
    while(gpio_get_level(ECHO_PIN) == 1)
    ;
    uint64_t end_time = esp_timer_get_time();

    float time_diff = (float) end_time - start_time;
    float distance = time_diff / 58.0;
    printf("Distancia: %0.1fcm\n", distance);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void measure_temp(void *pvParameters)
{
  
  ds18b20_init(DS_GPIO);
  while (1)
  {

    float temperatura = ds18b20_get_temp();
    printf("Temperature: %0.1f Graus\n", temperatura);
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}


void app_main() {
  // nvs_flash_init();
  // system_init();
  xTaskCreate(&measure_temp, "measure_temp", 2048, NULL, 5, NULL);
  xTaskCreate(&measure_distance, "measure_distance", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
  gpio_pad_select_gpio(TRIGGER_PIN);
  gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT );

  gpio_pad_select_gpio(ECHO_PIN);
  gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);


  // while(1) {
  //   printf("iniciando\n");

  //   gpio_set_level(TRIGGER_PIN, 1);
  //   ets_delay_us(10);
  //   gpio_set_level(TRIGGER_PIN, 0);
    
  //   printf("sinal de trigger enviado\n");

  //   while(gpio_get_level(ECHO_PIN) == 0)
  //   ;
  //   printf("recebendo echo...\n");

  //   uint64_t start_time = esp_timer_get_time();
  //   while(gpio_get_level(ECHO_PIN) == 1)
  //   ;
  //   printf("echo recebido\n");
  //   uint64_t end_time = esp_timer_get_time();

  //   float time_diff = (float) end_time - start_time;
  //   float distance = time_diff / 58.0;
  //   printf("START TIME: %d; END TIME: %d\n", start_time, end_time);
  //   printf("Distancia: %0.1fcm\n", distance);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
}