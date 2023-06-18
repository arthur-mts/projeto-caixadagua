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
#include "hal/gpio_types.h"
#include "rom/gpio.h"
#include "string.h"
#include "driver/gpio.h"
#include <freertos/queue.h>
#include <esp_intr_alloc.h>
#include "driver/i2c.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "ds18b20.h" //Include library
#define DS_GPIO 3    // GPIO where you connected ds18b20
#define TRIGGER_PIN 0
#define ECHO_PIN 1
#define RELE_MOTOR_PIN 18 //mudar
#define RELE_RESIST_PIN 12 //mudar
#include "display.c"
#define DISPLAY_SDA_PIN 4
#define DISPLAY_SCL_PIN 5
#define LCD_I2C_ADDR 0x27
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


void trigger_echo(void *pvParameters) {
  while(1) {
    gpio_set_level(TRIGGER_PIN, 1);
    ets_delay_us(10);
    gpio_set_level(TRIGGER_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void measure_temp(void *pvParameters) {
  ds18b20_init(DS_GPIO);
  while (1)
  {
    tempData = ds18b20_get_temp();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void IRAM_ATTR gpio_isr_handler_distance(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    int gpio_level = gpio_get_level(gpio_num);


    if (gpio_level == 0) {
      end_echo_time_check = esp_timer_get_time();
      float time_diff = (float) end_echo_time_check - start_echo_time_check;
      float distance = time_diff / 58.0;
      // TODO: checar se o valor ta maior ou menor doq o limite e marcar como is working 0
      distData.isWorking = 1;
      distData.distance = distance;
    } else {
      start_echo_time_check = esp_timer_get_time();
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

  xTaskCreate(&trigger_echo, "trigger_echo", 2048, NULL, 5, NULL);
}

void config_reles() {
  // Iniciando os reles desativados
  gpio_pad_select_gpio(RELE_MOTOR_PIN);
  gpio_set_direction(RELE_MOTOR_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(RELE_MOTOR_PIN, 1);

  gpio_pad_select_gpio(RELE_RESIST_PIN);
  gpio_set_direction(RELE_RESIST_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(RELE_RESIST_PIN, 1);
}

void config_measure_temp() {
  xTaskCreate(&measure_temp, "measure_temp", 1024, NULL, 5, NULL);
}

void enable_motor(int enabled) {
  gpio_set_level(RELE_MOTOR_PIN, !enabled);
}

void enable_resistencia(int enabled) {
  gpio_set_level(RELE_RESIST_PIN, !enabled);
}



void config_display() {
//   i2c_config_t i2c_config = {
//     .mode = I2C_MODE_MASTER,
//     .sda_io_num = DISPLAY_SDA_PIN,
//     .sda_pullup_en = GPIO_PULLUP_ENABLE,
//     .scl_io_num = DISPLAY_SCL_PIN,
//     .scl_pullup_en = GPIO_PULLUP_ENABLE,
//     .master.clk_speed = 100000
//   };

//   i2c_param_config(I2C_NUM_0, &i2c_config);
//   i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

//   uint8_t lcd_init_cmds[] = {
//     0x4E,   // Function Set: 2 linhas, 5x8 dots
//     0x01,   // Clear Display
//     0x0C,   // Display ON, Cursor OFF
//     0x06    // Entry Mode: Incrementa cursor
// };

//   i2c_cmd_handle_t cmd;
//   cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
//   i2c_master_write_byte(cmd, 0x00, true);

//   for (int i = 0; i < sizeof(lcd_init_cmds); i++) {
//       i2c_master_write_byte(cmd, lcd_init_cmds[i], true);
//   }

//   i2c_master_stop(cmd);
//   i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
//   i2c_cmd_link_delete(cmd);
  LCD_init(LCD_I2C_ADDR, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN, 16, 2);
}

void lcd_write_text(const char* text, uint8_t row, uint8_t col) {
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, 0x80 | (col + row * 0x40), true);

    const char* p = text;
    while (*p) {
        i2c_master_write_byte(cmd, *p++, true);
    }

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

void LCD_DemoTask(void* param)
{
    char num[20];
    while (true) {
        LCD_home();
        LCD_clearScreen();
        LCD_writeStr("16x2 I2C LCD");
        vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        LCD_writeStr("Lets Count 0-10!");
        vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        for (int i = 0; i <= 10; i++) {
            LCD_setCursor(8, 1);
            sprintf(num, "%d", i);
            LCD_writeStr(num);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
  
    }
}

void app_main() {
  config_measure_distance();
  config_reles();
  config_measure_temp();
  config_display();


  while(1) {
    ESP_LOGI(TAG, "Distancia: status=%i; value=%.2f", distData.isWorking, distData.distance);
    ESP_LOGI(TAG, "Temperatura: status=%i; value=%.2f", tempData.isWorking, tempData.temp);
    // lcd_write_text("Hello, World!", 0, 0);
    xTaskCreate(&LCD_DemoTask, "Demo Task", 2048, NULL, 5, NULL);

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}