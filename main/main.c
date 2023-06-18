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
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "ds18b20.h" //Include library
#define DS_GPIO 3    // GPIO where you connected ds18b20
#define TRIGGER_PIN 0
#define ECHO_PIN 1
#define RELE_MOTOR_PIN 18 //mudar
#define RELE_RESIST_PIN 12 //mudar
#include "driver/adc.h"
#define TAG "CAIXA DAGUA"

// Botoes
#define JOYSTICK_SW_PIN 12
#define JOYSTIC_Y_PIN 2

#define ADC_WIDTH_BIT 10
#define ADC_MAX_VALUE ((1<<ADC_WIDTH_BIT)-1)
#define DEFAULT_VREF    1100        // Valor de referência para a calibração ADC (em mV)
#define NO_OF_SAMPLES   64          // Número de amostras ADC para média

typedef enum display_status_t {
  READ = 0,
  WRITE_TEMP = 1,
  WRITE_NVL = 2
} display_status_t;

display_status_t display_status = READ;

typedef enum joystic_status {
  MIDDLE = 0,
  UP = 1,
  DOWN = 2
} joystic_status;

// DIsplay
#include "display.c"

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

static esp_adc_cal_characteristics_t adc1_chars;
void configure_buttons() {

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11));





  gpio_pad_select_gpio(JOYSTICK_SW_PIN);
  gpio_set_direction(JOYSTICK_SW_PIN, GPIO_MODE_INPUT);
}

int analogRead(uint8_t pin) {
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_2), &adc1_chars);
    return voltage;
}

joystic_status readJoystick() {
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_2), &adc1_chars);
  return voltage;

  if (voltage >= 2700) {
    return DOWN;
  }
  else if (voltage < 200) {
    return UP;
  } else {
    return MIDDLE;
  }
}

void display_menu() {
  write_on_lcd("TEMP | NVL | RD", 0);
  write_on_lcd(" 10C | 90ml|   ", 1);
}


void app_main() {
  config_measure_distance();
  config_reles();
  config_measure_temp();
  config_display();
  configure_buttons();


  display_menu();
  while(1) {
    ESP_LOGI(TAG, "Eixo y: %i", analogRead(JOYSTIC_Y_PIN));
    // ESP_LOGI(TAG, "Distancia: status=%i; value=%.2f", distData.isWorking, distData.distance);
    // ESP_LOGI(TAG, "Temperatura: status=%i; value=%.2f", tempData.isWorking, tempData.temp);
    vTaskDelay(1000 / portTICK_PERIOD_MS);  
  }
}