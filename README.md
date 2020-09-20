# BME280 driver component for the ESP-IDF
This component includes the BME280 driver from BoschSensortec as a submodule
and an I2C HAL to use this driver on the ESP-IDF.

It should work with ESP-IDF major versions 3 and 4.

## Prerequisites & Limitations
The used I2C port needs to be properly initialized before using the driver.

The driver is not threadsafe, so calls to the driver without protection should
only be done within a single task at a time. The HAL configuration however does
support setting a mutex to protect the I2C transactions it does. This is useful
when another library needs to access another device on the same bus, since I2C
port operations on the same bus are not threadsafe.

## Examples
An [example][1] for the usage of this component can be found in the `examples`
directory of this repository.

Since the usage of the BME280 driver is not straightforward and finding the
right parameters may be complicated, a look into the [documentation][2]
of the BME280 is recommended. It also does contain recommended settings for
different usage scenarios.

[1]: examples/i2c_example.c
[2]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf