#include "bme280_esp_hal_i2c.h"
#include "assert.h"
#include "rom/ets_sys.h"
#include "freertos/task.h"

/**
 * Convert `esp_err_t` errors returned by `i2c_master_cmd_begin()` to
 * `bme280_esp_hal_err` type errors.
 * @param esp_err Error to convert.
 * @return The converted error.
 */
static bme280_esp_hal_err bme280_esp_hal_err_from_esp_err(
    const esp_err_t esp_err);

static bme280_esp_hal_err bme280_esp_hal_err_from_esp_err(
    const esp_err_t esp_err) {
  bme280_esp_hal_err err;
  switch (esp_err) {
    case ESP_OK:
      err = BME280_ESP_HAL_OK;
      break;
    case ESP_FAIL:
      err = BME280_ESP_HAL_ERR_I2C_DEVICE;
      break;
    case ESP_ERR_TIMEOUT:
      err = BME280_ESP_HAL_ERR_I2C_TIMEOUT;
      break;
    default:
      err = BME280_ESP_HAL_ERR_CONFIG;
      break;
  }
  return err;
}

bme280_esp_hal_err bme280_esp_i2c_scan_addr(bme280_esp_hal_i2c_intf* intf) {
  bme280_esp_hal_err err = BME280_ESP_HAL_ERR_I2C_TIMEOUT;
  uint8_t addr_list[] = BME280_ESP_HAL_SCAN_I2C_ADDRS;

  if ((!intf->i2c_mutex) ||
      xSemaphoreTake(intf->i2c_mutex, intf->i2c_mutex_timeout_ms) == pdTRUE) {
    for (int i = 0; i < sizeof(addr_list); ++i) {
      uint8_t addr = addr_list[i];

      i2c_cmd_handle_t cmd = i2c_cmd_link_create();

      ESP_ERROR_CHECK(i2c_master_start(cmd));
      ESP_ERROR_CHECK(
          i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true));
      ESP_ERROR_CHECK(i2c_master_stop(cmd));

      esp_err_t esp_err = i2c_master_cmd_begin(
          intf->i2c_port, cmd, pdMS_TO_TICKS(intf->i2c_timeout_ms));
      err = bme280_esp_hal_err_from_esp_err(esp_err);

      i2c_cmd_link_delete(cmd);

      if (err != BME280_ESP_HAL_ERR_I2C_TIMEOUT) {
        if (err == BME280_ESP_HAL_OK)
          intf->i2c_addr = addr;
        break;
      }
    }

    if (intf->i2c_mutex)
      assert(xSemaphoreGive(intf->i2c_mutex) == pdTRUE);

  } else {
    err = BME280_ESP_HAL_ERR_I2C_MUTEX_TIMEOUT;
  }

  return err;
}

void bme280_esp_hal_delay_us(uint32_t period, void* intf_ptr) {
  /** If the period to wait is smaller than the task tick period, use
   * ets_delay_us(), otherwise vTaskDelay().
   */
  if (period < (portTICK_PERIOD_MS * 1000)) {
    ets_delay_us(period);
  } else {
    /** Calculate the required waiting time in ticks and round up to a full
     * tick. */
    vTaskDelay(((period - 1) / (portTICK_PERIOD_MS * 1000)) + 1);
  }
}

bme280_esp_hal_err bme280_esp_hal_i2c_read(uint8_t reg_addr,
                                           uint8_t* reg_data,
                                           uint32_t len,
                                           void* intf_ptr) {
  bme280_esp_hal_err err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  const bme280_esp_hal_i2c_intf* intf = (const bme280_esp_hal_i2c_intf*)intf_ptr;

  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(i2c_master_write_byte(
      cmd, (intf->i2c_addr << 1) | I2C_MASTER_WRITE, true));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
  /**
   * The template states here should be a stop. But the sensor will not
   * answer when it is present, so the stop is omitted.
   * ESP_ERROR_CHECK(i2c_master_stop(cmd));
   */
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(i2c_master_write_byte(
      cmd, (intf->i2c_addr << 1) | I2C_MASTER_READ, true));
  ESP_ERROR_CHECK(i2c_master_read(cmd, reg_data, len, I2C_MASTER_LAST_NACK));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));

  if ((!intf->i2c_mutex) ||
      xSemaphoreTake(intf->i2c_mutex, intf->i2c_mutex_timeout_ms) == pdTRUE) {
    esp_err_t esp_err = i2c_master_cmd_begin(
        intf->i2c_port, cmd, pdMS_TO_TICKS(intf->i2c_timeout_ms));
    err = bme280_esp_hal_err_from_esp_err(esp_err);

    if (intf->i2c_mutex)
      assert(xSemaphoreGive(intf->i2c_mutex) == pdTRUE);
  } else {
    err = BME280_ESP_HAL_ERR_I2C_MUTEX_TIMEOUT;
  }

  i2c_cmd_link_delete(cmd);

  return err;
}

bme280_esp_hal_err bme280_esp_hal_i2c_write(uint8_t reg_addr,
                                            const uint8_t* reg_data,
                                            uint32_t len,
                                            void* intf_ptr) {
  bme280_esp_hal_err err = BME280_ESP_HAL_OK;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  const bme280_esp_hal_i2c_intf* intf = (const bme280_esp_hal_i2c_intf*)intf_ptr;

  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(i2c_master_write_byte(
      cmd, (intf->i2c_addr << 1) | I2C_MASTER_WRITE, true));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
  ESP_ERROR_CHECK(i2c_master_write(cmd, (uint8_t*)reg_data, len, true));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));

  if ((!intf->i2c_mutex) ||
      xSemaphoreTake(intf->i2c_mutex, intf->i2c_mutex_timeout_ms) == pdTRUE) {
    esp_err_t esp_err =
        i2c_master_cmd_begin(intf->i2c_port, cmd, intf->i2c_timeout_ms);
    err = bme280_esp_hal_err_from_esp_err(esp_err);
    if (intf->i2c_mutex)
      assert(xSemaphoreGive(intf->i2c_mutex) == pdTRUE);
  } else {
    err = BME280_ESP_HAL_ERR_I2C_MUTEX_TIMEOUT;
  }

  i2c_cmd_link_delete(cmd);

  return err;
}