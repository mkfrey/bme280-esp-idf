/**
 * ESP-IDF I2C HAL for the Bosch BME280 driver.
 *
 * @file bme280_esp_hal_i2c.h
 */

#ifndef _BME280_ESP_HAL_I2C
#define _BME280_ESP_HAL_I2C

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_types.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/** Possible I2C device addresses to scan for when using the scan helper
 * function. */
#define BME280_ESP_HAL_SCAN_I2C_ADDRS \
  { 0x76, 0x77 }

/** The I2C port of the default configuration. */
#define BME280_ESP_HAL_DEFAULT_I2C_PORT I2C_NUM_0

/** The I2C device address of the default configuration. BME280 uses either 0x76
 * or 0x77. */
#define BME280_ESP_HAL_DEFAULT_I2C_ADDR 0x76

/** The I2C timeout of the default configuration */
#define BME280_ESP_HAL_DEFAULT_I2C_TIMEOUT_MS 500

/** The mutex timeout of the default configuration */
#define BME280_ESP_HAL_DEFAULT_MUTEX_TIMEOUT_MS 500

/* The default I2C HAL configuration. */
#define BME280_ESP_HAL_DEFAULT_I2C_INTF                              \
  {                                                                  \
    .i2c_port = BME280_ESP_HAL_DEFAULT_I2C_PORT,                     \
    .i2c_addr = BME280_ESP_HAL_DEFAULT_I2C_ADDR,                     \
    .i2c_timeout_ms = BME280_ESP_HAL_DEFAULT_I2C_TIMEOUT_MS,         \
    .i2c_mutex_timeout_ms = BME280_ESP_HAL_DEFAULT_MUTEX_TIMEOUT_MS, \
    .i2c_mutex = NULL                                                \
  }

/**
 * I2C HAL configuration struct.
 * @note This struct is passed to the driver by pointer. Be aware of its
 * lifetime.
 */
typedef struct {
  /** I2C port to use. */
  i2c_port_t i2c_port;
  /** I2C address to use. */
  uint8_t i2c_addr;
  /** I2C transaction timeout in milliseconds. */
  uint16_t i2c_timeout_ms;
  /** Maximum time to wait for I2C mutex to be taken. Has no effect if i2c_mutex
   * is NULL. */
  uint16_t i2c_mutex_timeout_ms;
  /** I2C mutex to make I2C transactions of other libraries communicating with
   * other devices on the same I2C port threadsafe. Set to NULL if this is not
   * required. */
  SemaphoreHandle_t i2c_mutex;
} bme280_esp_hal_i2c_intf;

/**
 * HAL operation error type.
 */
typedef int8_t bme280_esp_hal_err;
/** No error occured. */
#define BME280_ESP_HAL_OK 0
/** The I2C transaction timed out, bus was busy or no device did respond. */
#define BME280_ESP_HAL_ERR_I2C_TIMEOUT 1
/** Taking the I2C mutex timed out. */
#define BME280_ESP_HAL_ERR_I2C_MUTEX_TIMEOUT 2
/** The BME280 did not ACK the packets. */
#define BME280_ESP_HAL_ERR_I2C_DEVICE 3
/** General configuration error (e.g. I2C driver not installed). */
#define BME280_ESP_HAL_ERR_CONFIG 4

/**
 * Helper function scanning the I2C bus to determine the sensor address.
 * @param[in,out] intf Pointer to an I2C HAL configuration. All fields
 * except `i2c_addr` need to be set. `i2c_addr` is being set by this
 * function if the scan is successful.
 * @return `BME280_ESP_HAL_OK`, if the scan was successful, one of
 * `BME280_ESP_HAL_ERR_*` otherwise.
 */
bme280_esp_hal_err bme280_esp_i2c_scan_addr(bme280_esp_hal_i2c_intf* intf);

/**
 * I2C HAL function delaying the task by a given period of time.
 * Needs to be assigned to bme280_dev::delay_us.
 * @param period Delay period in microseconds (us).
 * @param intf_ptr Pointer to a bme280_esp_hal_i2c_intf configuration.
 */
void bme280_esp_hal_delay_us(uint32_t period, void* intf_ptr);

/**
 * I2C HAL function reading a given amount of bytes from a given register.
 * @param[in] reg_addr Register address to read from.
 * @param[out] reg_data Pointer to a buffer the read data is written to.
 * @param[in] len Amount of bytes to read.
 * @param[in] intf_ptr Pointer to a bme280_esp_hal_i2c_intf configuration.
 * @
 */
int8_t bme280_esp_hal_i2c_read(uint8_t reg_addr,
                               uint8_t* reg_data,
                               uint32_t len,
                               void* intf_ptr);

/**
 * I2C HAL function writing a given amount of bytes to a given register.
 * @param[in] reg_addr Register address to write tp.
 * @param[out] reg_data Pointer to a buffer with the data to write.
 * @param[in] len Amount of bytes to write.
 * @param[in] intf_ptr Pointer to a bme280_esp_hal_i2c_intf configuration.
 */
int8_t bme280_esp_hal_i2c_write(uint8_t reg_addr,
                                const uint8_t* reg_data,
                                uint32_t len,
                                void* intf_ptr);

#ifdef __cplusplus
}
#endif

#endif  //_BME280_ESP_HAL_I2C