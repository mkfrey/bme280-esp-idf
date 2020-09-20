/* BME280 driver usage example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <esp_types.h>
#include <stdio.h>
#include "bme280.h"
#include "bme280_esp_hal_i2c.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** I2C port settings. */
#define BME280_I2C_PORT I2C_NUM_0
#define BME280_I2C_GPIO_NUM_CLOCK GPIO_NUM_32
#define BME280_I2C_GPIO_NUM_DATA GPIO_NUM_33
#define BME280_I2C_FREQ \
  1000000 /* BME280 supports up to 10MHz, but ESP32 maxes out at 1 MHz. */

/** Measurement printing interval. */
#define BME280_PRINT_INTERVAL_MS 5000

static bme280_esp_hal_i2c_intf bme280_hal_conf =
    BME280_ESP_HAL_DEFAULT_I2C_INTF;

static struct bme280_dev bme280 = {.intf_ptr = &bme280_hal_conf,
                                   .intf = BME280_I2C_INTF,
                                   .read = bme280_esp_hal_i2c_read,
                                   .write = bme280_esp_hal_i2c_write,
                                   .delay_us = bme280_esp_hal_delay_us};

static void init_i2c() {
  i2c_config_t i2c = {.mode = I2C_MODE_MASTER,
                      .sda_io_num = BME280_I2C_GPIO_NUM_DATA,
                      .scl_io_num = BME280_I2C_GPIO_NUM_CLOCK,
                      .sda_pullup_en = true,
                      .scl_pullup_en = true,
                      .master.clk_speed = BME280_I2C_FREQ};
  ESP_ERROR_CHECK(i2c_param_config(BME280_I2C_PORT, &i2c));
  ESP_ERROR_CHECK(i2c_driver_install(BME280_I2C_PORT, I2C_MODE_MASTER, 0, 0,
                                     ESP_INTR_FLAG_IRAM));
}

void bme_task(void* pvParameters) {
  for (;;) {
    struct bme280_data sensor_data;
    assert(bme280_get_sensor_data(BME280_ALL, &sensor_data, &bme280) ==
           BME280_OK);

    printf(
        "----------------------\n"
        "Temperature: %7.2f °C\n"
        "Humidity: %9.2f %%\n"
        "Pressure: %9.2f Pa\n"
        "----------------------\n",
        sensor_data.temperature, sensor_data.humidity, sensor_data.pressure);

    vTaskDelay(pdMS_TO_TICKS(BME280_PRINT_INTERVAL_MS));
  }
}

void app_main() {
  /** Initialize the I2C port. */
  init_i2c();

  /** Change the BME280 HAL settings. */
  bme280_hal_conf.i2c_port = BME280_I2C_PORT;

  /** Scan for the BME280 sensor address. */
  assert(bme280_esp_i2c_scan_addr((bme280_esp_hal_i2c_intf*)bme280.intf_ptr) ==
         BME280_ESP_HAL_OK);

  /** Initialize the BME280. */
  assert(bme280_init(&bme280) == BME280_OK);

  /** Set the settings to the recommended values for indoor navigation*/
  bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
  bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
  bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
  bme280.settings.filter = BME280_FILTER_COEFF_16;

  assert(bme280_set_sensor_settings(
             BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL,
             &bme280) == BME280_OK);

  /** Put the sensor from sleep to normal operation mode. */
  assert(bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280) == BME280_OK);

  /** Wait for the sensor to run its first measurement. */
  vTaskDelay(10);

  /** Start the periodic measurment data printing task. */
  assert(xTaskCreatePinnedToCore(bme_task, "bme", 2048, NULL, 0, NULL, 1) ==
         pdTRUE);
}
